use rclrs::*;
use std::{sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;

struct SimplePublisher {
    publisher: Arc<Publisher<StringMsg>>,
}

impl SimplePublisher {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("publisher_node")?;

        // Declare and get topic parameter
        let topic: Arc<str> = node.declare_parameter("topic")
            .default(Arc::from("default_topic"))  // Use String instead of &str
            .mandatory()
            .unwrap()
            .get();
        
        let publisher = node.create_publisher::<StringMsg>(&topic)?;
        Ok(Self { publisher })
    }

    fn publish_data(&self, prefix: &str, suffix: &str, count: i32) -> Result<(), RclrsError> {
        let msg = StringMsg {
            data: format!("{} {} {}", prefix, count, suffix),
        };
        println!("Publishing: {}", msg.data);
        self.publisher.publish(msg)
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let publisher = Arc::new(SimplePublisher::new(&executor)?);
    let publisher_clone = Arc::clone(&publisher);

    // Create parameter node and declare parameters
    let node = executor.create_node("parameter_node")?;
    
    // Declare parameters with defaults
    let publish_rate_ms: u64 = node.declare_parameter("publish_rate_ms")
        .default(1000)
        .mandatory()
        .unwrap()
        .get() as u64;
    
    let prefix: Arc<str> = node.declare_parameter("prefix")
        .default(Arc::from("Hello"))  // Use String instead of &str
        .mandatory()
        .unwrap()
        .get();
    
    let suffix: Arc<str> = node.declare_parameter("suffix")
        .default(Arc::from("World"))  // Use String instead of &str
        .mandatory()
        .unwrap()
        .get();
    
    let mut count: i32 = node.declare_parameter("count_start")
        .default(0)
        .mandatory()
        .unwrap()
        .get() as i32;

    thread::spawn(move || {
        loop {
            if let Err(e) = publisher_clone.publish_data(&prefix, &suffix, count) {
                eprintln!("Error publishing: {:?}", e);
            }
            count += 1;
            thread::sleep(Duration::from_millis(publish_rate_ms));
        }
    });

    executor.spin(SpinOptions::default()).first_error()
}