use ros_z::{Builder, Result, context::ZContextBuilder};
use ros_z_msgs::std_msgs::String as RosString;

#[tokio::main]
async fn main() -> Result<()> {
    let domain_id: usize = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let ctx = ZContextBuilder::default()
        .with_domain_id(domain_id)
        .build()?;

    let node = ctx.create_node("rust_listener").build()?;
    let subscriber = node.create_sub::<RosString>("/talker").build()?;

    println!("Listening on /talker ...");

    loop {
        let msg = subscriber.async_recv().await?;
        println!("I heard: [{}]", msg.data);
    }
}
