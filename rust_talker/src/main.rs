use std::time::Duration;
use ros_z::{Builder, Result, context::ZContextBuilder};
use ros_z_msgs::std_msgs::String as RosString;

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize ros-z context (connects to router on localhost:7447)
    let domain_id: usize = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let ctx = ZContextBuilder::default()
        .with_domain_id(domain_id)
        .build()?;

    // Create a ROS 2 node
    let node = ctx.create_node("talker").build()?;

    // Create a publisher for the /chatter topic
    let pub_handle = node.create_pub::<RosString>("/talker").build()?;

    // Publish messages every second
    let mut count = 0;
    loop {
        let msg = RosString {
            data: format!("Hello from ros-z #{}", count),
        };
        println!("Publishing: {}", msg.data);
        pub_handle.async_publish(&msg).await?;

        count += 1;
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}
