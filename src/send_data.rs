use std::{net::SocketAddr, path::Path};

use anyhow::{Context, Result};
use bytes::Bytes;
use s2n_quic::{client::Connect, Client};
use serde::Serialize;

use crate::{operate_pcd::PointXYZ};


#[derive(Clone, Debug, Serialize)]
pub struct PointCloudPacket {
    pub count: usize,
    pub is_cr: bool,
    pub cr_points_num: usize,
    pub cr_points: Vec<PointXYZ>,
    pub is_lm: bool,
    pub lm_points_num: usize,
    pub lm_points: Vec<PointXYZ>,
}

pub async fn send_via_quic(
    data: PointCloudPacket,
    server_addr: &str,
    server_name: &str,
) -> Result<()> {
    // let start = std::time::Instant::now();

    // クライアントのアドレス
    let local_address: SocketAddr = "0.0.0.0:0".parse()?;

    // IO プロバイダーの設定
    let io = s2n_quic::provider::io::Default::builder()
        .with_max_mtu(1228)?
        .with_receive_address(local_address)?
        .build()?;

    // クライアントの作成
    let client = Client::builder()
        .with_tls(Path::new("./certs/ca-cert.pem"))?
        .with_io(io)?
        .start()
        .context("Failed to start client")?;

    log::debug!("Client started on {}", client.local_addr()?);

    // サーバーへの接続
    let addr: SocketAddr = server_addr.parse()
        .context("Failed to parse server address")?;
    let connect = Connect::new(addr).with_server_name(server_name);
    
    let mut connection = client.connect(connect).await
        .context("Failed to connect to the server")?;

    let start = std::time::Instant::now();

    connection.keep_alive(true)?;

    // ストリームを開く
    let stream = connection.open_bidirectional_stream().await?;
    let (mut receive_stream, mut send_stream) = stream.split();

    // データ送信
    let send_data = bincode::serialize(&data).context("Failed to transform the data to binary")?;
    log::debug!("Serialized data size: {} bytes", send_data.len());
    send_stream.send(Bytes::from(send_data)).await
        .context("Failed to send the data")?;
    log::debug!("Sent the data to the server");

    // 送信完了
    send_stream.finish()
        .context("Failed to finish sending")?;
    log::debug!("Finished sending the data to the server");

    // サーバーからのレスポンスを受信
    while let Ok(Some(response)) = receive_stream.receive().await {
        log::debug!("Received response: {:?}", response);
    }

    // コネクションを閉じる
    connection.close(0u32.into());
    log::debug!("Closed the connection");

    let duration = start.elapsed();
    log::debug!("Elapsed time: {:?}", duration);

    Ok(())
}