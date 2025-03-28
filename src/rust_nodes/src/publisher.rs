use rclrs::*;
use std::{sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;

struct SimplePublisher {
    publisher: Arc<Publisher<StringMsg>>,
}

impl SimplePublisher {
    // Create a publisher node using the executor's create_node method.
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        // Create a node with name "publisher_node"
        let node = executor.create_node("publisher_node")?;
        // The topic name is provided as the first command-line parameter.
        let topic = std::env::args().nth(1).unwrap_or_else(|| "default_topic".into());
        let publisher = node.create_publisher::<StringMsg>(&topic)?;
        Ok(Self { publisher })
    }

    // Publish a message with a given prefix, count, and suffix.
    fn publish_data(&self, prefix: &str, suffix: &str, count: i32) -> Result<(), RclrsError> {
        let msg = StringMsg {
            data: format!("{} {} {}", prefix, count, suffix),
        };
        self.publisher.publish(msg)
    }
}

fn main() -> Result<(), RclrsError> {
    // Expect command-line parameters:
    //   <executable> <topic_name> <publish_rate_ms> <prefix> <suffix> <count_start>
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 6 {
        eprintln!(
            "Usage: {} <topic_name> <publish_rate_ms> <prefix> <suffix> <count_start>",
            args[0]
        );
        return Ok(());
    }
    // Note: the first parameter (topic_name) is used inside SimplePublisher.
    let publish_rate_ms: u64 = args[2].parse().unwrap_or(1000);
    let prefix = args[3].clone();
    let suffix = args[4].clone();
    let mut count: i32 = args[5].parse().unwrap_or(0);

    // Create a ROS 2 context and executor.
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let publisher = Arc::new(SimplePublisher::new(&executor)?);
    let publisher_clone = Arc::clone(&publisher);

    // Spawn a thread to publish messages periodically.
    thread::spawn(move || {
        loop {
            if let Err(e) = publisher_clone.publish_data(&prefix, &suffix, count) {
                eprintln!("Error publishing: {:?}", e);
            }
            count += 2;
            thread::sleep(Duration::from_millis(publish_rate_ms));
        }
    });

    // Spin the executor to process callbacks.
    executor.spin(SpinOptions::default()).first_error()
}
