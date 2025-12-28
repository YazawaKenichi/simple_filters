# simple_filters

## Overview
フィルタをかますだけのパッケージ

## Usage
## Prerequisites
- ROS2 Humble
- C++ 17

### Download and Install
1. ワークスペースにこのパッケージをクローン
    ``` cmd
    cd ~/ros2_ws/src/
    git clone https://github.com/yazawakenichi/simple_filters
    ```
2. パッケージのビルド
    ``` cmd
    cd ~/ros2_ws/
    colcon build --packages-select simple_filters --symlink-install
    ```
3. 実行ファイルのオーバーレイ
    ``` cmd
    source install/local_setup.bash
    ```

### Launching Node
`ros2 launch` を用いることでノードを起動することができます。

新しくターミナルを開いた場合、ノードを起動する前に `source install/local_setup.bash` することを忘れないでください。

``` cmd
ros2 launch simple_filters simple_filters.launch.py
```

## Package Description

#### Node
|Name|Type|Description
|---|---|---
|``|| ... するノード
|``|| ... するノード
|``|| ... するノード
|``|| ... するノード

#### Topic
|Name|Type|Description
|---|---|---
|``|`std_msgs::msg::`| ...
|``|`std_msgs::msg::`| ...

### Parameters
|Name|Type|Default|Description
|---|---|---|---
|`/pub_topic_name`|`std::string`|`"pub"`| パブリッシュするトピック
|`/sub_topic_name`|`std::string`|`"sub"`| サブスクライブするトピック

## License
このソフトウェアは、MIT License の下、再頒布および使用が許可されます。

(C) 2025 YAZAWA Kenichi

