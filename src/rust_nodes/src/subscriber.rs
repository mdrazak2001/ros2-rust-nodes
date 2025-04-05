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

        // Declare and get topic parameter
        let topic: Arc<str> = node.declare_parameter("topic")
            .default(Arc::from("default_topic"))  // Use String instead of &str
            .mandatory()
            .unwrap()
            .get();

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
    // Create a ROS 2 context and executor.
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let subscriber = Arc::new(SimpleSubscriptionNode::new(&executor)?);
    let subscriber_clone = Arc::clone(&subscriber);

    // Create parameter node and declare parameters
    let node = executor.create_node("parameter_node")?;

    let output_file: Arc<str> = node.declare_parameter("output_file")
        .default(Arc::from("subscriber_output.txt"))
        .mandatory()
        .unwrap()
        .get();

    // Spawn a thread that periodically processes received messages and writes them to the file.
    thread::spawn(move || loop {
        subscriber_clone.process_data(&output_file);
        thread::sleep(Duration::from_millis(1000));
    });

    // Spin the executor to process incoming messages.
    executor.spin(SpinOptions::default()).first_error()
}
