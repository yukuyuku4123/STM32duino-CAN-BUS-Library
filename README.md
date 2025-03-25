# STM32duino CAN BUS Library

STM32duino用のCAN busライブラリです。

# 対応マイコン

- STM32F3シリーズ

今後順次追加予定

# How to use?

main.cpp内に`#include "BUS.h"`を記述してインクルードします。  
`BUS 名前;`と記述してCANクラスをインスタンス化させることで使用できるようになります。  

## Functions and Variables
基本的にはArduinoのSerialと同じように使用することが出来ます。

### `begin(BitRate);`

CAN通信を引数に渡したビットレートで開始します。  
引数に渡せるものは`_1M`または`1000000`、`_500K`または`500000`、`_250K`または`250000`です。それ以外の整数を与えると125kbpsでCAN通信が開始されます。  
現在選択できるビットレートは1Mbps、500kbps、250kbps、125kbpsです。  

**注)仕様の変更予定あり**

### `setID(ID1, ID2);`

引数に渡したIDでフィルターをリストモードで設定することが出来ます。  
現在は2個まで設定できます。  

**注)仕様の変更予定あり**

### `setMask(ID, Mask);`

引数に渡したIDとマスクでフィルターをマスクモードで設定することが出来ます。  

**注)仕様の変更予定あり**

### `write(ID, Data, Length);`

引数に渡した値を元にCANメッセージが送信されます。  
渡した引数は`CAN_Tx_Msg`に自動で格納されます。  

### `available();`

CANのFIFOにメッセージが届いているか確認します。  
届いているメッセージ数が返り値です。何も無ければ0を返します。  

### `read();`

FIFOに届いているメッセージを読みだします。  
読みだしたメッセージは`CAN_Rx_Msg`に自動で格納されます。  

### `CAN_Msg`

CANのメッセージに関する情報を格納する構造体です。  
`CAN_Tx_Msg`、`CAN_Rx_Msg`が宣言されています。  

```c++
struct CAN_Msg {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
    };
```

**注)仕様の変更予定あり**

