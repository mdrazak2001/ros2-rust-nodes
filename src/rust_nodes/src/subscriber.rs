use rclrs::*;
use std::{
    fs::OpenOptions,
    io::Write,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};
use std_msgs::msg::String as StringMsg;

struct SimpleSubscriptionNode {
    _subscription: Arc<Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}

impl SimpleSubscriptionNode {
    // Create a subscriber node using the executor's create_node method.
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("subscriber_node")?;
        // The topic name is provided as the first command-line parameter.
        let topic = std::env::args().nth(1).unwrap_or_else(|| "default_topic".into());
        let data: Arc<Mutex<Option<StringMsg>>> = Arc::new(Mutex::new(None));
        let data_clone = Arc::clone(&data);
        let subscription = node.create_subscription::<StringMsg, _>(
            &topic,
            move |msg: StringMsg| {
                *data_clone.lock().unwrap() = Some(msg);
            },
        )?;
        Ok(SimpleSubscriptionNode {
            _subscription: subscription,
            data,
        })
    }

    // Process the latest message and write it to the provided file.
    fn process_data(&self, output_file: &str) {
        if let Some(ref msg) = *self.data.lock().unwrap() {
            println!("Received: {}", msg.data);
            if let Ok(mut file) = OpenOptions::new()
                .create(true)
                .append(true)
                .open(output_file)
            {
                if let Err(e) = writeln!(file, "Received: {}", msg.data) {
                    eprintln!("Failed to write to file: {:?}", e);
                }
            } else {
                eprintln!("Failed to open file: {}", output_file);
            }
        } else {
            println!("No message received yet.");
        }
    }
}

fn main() -> Result<(), RclrsError> {
    // Expect command-line parameters:
    //   <executable> <topic_name> <output_file>
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: {} <topic_name> <output_file>", args[0]);
        return Ok(());
    }
    // The first parameter (topic_name) is used in subscription creation.
    let output_file = args[2].clone();

    // Create a ROS 2 context and executor.
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let subscriber = Arc::new(SimpleSubscriptionNode::new(&executor)?);
    let subscriber_clone = Arc::clone(&subscriber);

    // Spawn a thread that periodically processes received messages and writes them to the file.
    thread::spawn(move || loop {
        subscriber_clone.process_data(&output_file);
        thread::sleep(Duration::from_millis(1000));
    });

    // Spin the executor to process incoming messages.
    executor.spin(SpinOptions::default()).first_error()
}
