# フラクタル: 模様や三次元構造のフラクタル次元を推定するソフトウエアアーキテクチャー
ステータス：プロトタイプ  
飯田一鑑  

*英語版を閲覧: [English version (main page)](README.md)*

**フラクタル**とはボックスカウント法を使用して一般的な模様や形のフラクタル次元を推定するコマンドライン専用ツールであり、[こちら](https://www.mathworks.com/matlabcentral/fileexchange/13063-boxcount/content/boxcount/html/demo.html)のMATLAB用のプログラムに基づいて作成されました。

## 目次
- [概要](#概要)  
	- [基本的な用途](#基本的な用途)
	- [入力データが二次元の場合](#入力データが二次元の場合)
	- [入力データが三次元の場合](#入力データが三次元の場合)
	- [プログラムの表とその役割](#プログラムの表とその役割)
- [必須ライブラリ](#必須ライブラリ)  
- [インストール法](#インストール法)  
- [使い方](#使い方)  
  - [画像内の模様のフラクタル解析を行う](＃画像内の模様のフラクタル解析を行う)
  - [ASCIIファイルをPCDファイルに変換](#asciiファイルをpcdファイルに変換)
  - [三次元データ専用フラクタル解析コアプログラムを使う](#三次元データ専用フラクタル解析コアプログラムを使う)
  - [PCLを使用してポイントクラウドファイルを視覚化](#pclを使用してポイントクラウドファイルを視覚化)
  - [ヘルプメニューを引き出す](#ヘルプメニューを引き出す)
- [使用例](#使用例)
- [ご質問のある方](#ご質問のある方) 

## 概要

### 基本的な用途

| 入力データの次元 | 主要機能 | その他 | 
| :------------: | :-----:| :----: |
| 二次元（画像）| フラクタル解析 | 無し |
| 三次元（ポイントクラウド）| フラクタル解析 | ポイントクラウドの視覚化 | 

### 入力データが二次元の場合

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/2d_flowchart_ja.png "二次元インプット")

### 入力データが三次元の場合

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/3d_flowchart_ja.png "三次元インプット")

### プログラムの役割表


| プログラム名 | 使用言語 | 備考                     | 
| :--------: | :-----: | :----------------------: |
| fractal2d | C++ | 二次元データ（画像）専用のフラクタル解析コアプログラム |
| fractal3d | C++ | 三次元データ専用のフラクタル解析コアプログラム | 
| visualize | C++ | ポイントクラウドを視覚化するプログラム | 
| ascii2pcd | C++ | PCLが扱えるPCDファイルをASCIIポイントクラウドファイルから作成するプログラム | 
| plot_fractal.py | Python | フラクタル解析の結果をもとにフラクタル次元の推定グラフを作るプログラム | 
| plot_fractal.m | MATLAB | 上記のプログラムと同じく、MATLAB用のプログラム | 
| visualizeDataset.m | MATLAB | 特定されたポイントクラウドファイルを視覚化するMATLAB用のプログラム（[コンピュータビジョンツールボックス](http://jp.mathworks.com/products/computer-vision/index.html?s_tid=gn_loc_drop)が必要）|

## 必須ライブラリ  
* CMake（バージョン2.8以後）
* PCL (ポイントクラウドライブラリ)
* OpenCV
* Eigen
* Boost
* PythonかMATLAB (プロット作成のため）

## インストール法  
**以下のインストール法はUbuntu 14.04でしか確認されておりません。**
* Github上のレポジトリをコンピューターにクローンする。
```bash 
$ git clone https://github.com/kaziida24/fractal
```
* 先ほどクローンした"fractal"というフォルダの中に移動して、"build"というフォルダを作成し、"build"の中に移動する。
```bash
$ cd fractal 
$ mkdir build
$ cd build 
```
* CMakeを用いてMakefileを作成し、PythonとMATLABのスクリプトをコピーする。
```bash
$ cmake ..
```
* *make* でプログラムをコンパイルする。
```bash 
$ make all 
```

## 使い方

### 画像内の模様のフラクタル解析を行う

*fractal2d*プログラムと同時に解析する画像のファイル名を入力してください。解析結果をまとめたファイルはプログラムが終わり次第、自動的に書き込まれます。この場合、解析結果ファイル名は*image_output.txt*です。

```bash
$ ./fractal2d image.jpg
```

### ASCIIファイルをPCDファイルに変換

*ascii2pcd*プログラムと同時にASCIIファイル名を入力し、新しくPCDファイルが作成されます。
```bash
$ ./ascii2pcd filename.txt
```
こちらのコマンドは*filename.txt*というASCIIファイルから*filename.pcd*というPCDファイルを作成します。

### 三次元データ専用フラクタル解析コアプログラムを使う

*fractal*プログラムと同時に解析を行うPCDファイル名を入力します。
```bash
$ ./fractal filename.pcd
```
この場合、フラクタル解析の結果は*filename_output.txt*というファイルに保存されます。

### PCLを使用してポイントクラウドファイルを視覚化

*visualize*プログラムと同時に視覚化するポイントクラウドファイル名を入力します。
```bash
$ ./visualize filename.pcd
```

### ヘルプメニューを引き出す

全てのプログラムは*-h*を同時に入力すると案内画面が表示されます。  

*fractal*の案内画面
```bash
$ ./fractal -h
```
*visualize*の案内画面
```bash
$ ./visualize -h
```

*ascii2pcd*の案内画面
```bash
$ ./ascii2pcd -h
```

## 使用例

これから下の画像内の模様のフラクタル次元を推定する手順をご紹介します。

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/images/fractal_img.png "サンプル画像")

初めに、解析する画像を全て*images*のフォルダーに保存してください。*fractal2d*は特定された画像ファイルをこのフォルダー内から探します。プログラムを実行する際、解析する画像のファイル名も打ち込むことを忘れないようご注意ください。この場合、ファイル名は**fractal_img.png**です。

```bash
$ ./fractal2d fractal_img.png 
```

プログラム実行後、画面上にさらに指示が表示されます。次に進むためにスペースキーを押して、解析完了 (analysis complete)が出るまで画面上の指示に従ってください。

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/fractal2d_example1.png)

フラクタル解析終了後、Pythonで直接ポストプロセスする選択があります。ポストプロセスすると下のグラフが作成されます。

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/test1_loglog.png) ![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/test2_slope.png)

上の方のグラフはフラクタル解析で作成される典型的なプロットであり、ボックスカウント法により定められたボックスの大きさと数の関係を表しています。ボックスの大きさは指数関数上に変化するため、対数目盛でグラフを作成すると直線となります。下の方のグラフは直線の勾配（傾き）を表しており、この値が画像内の模様のフラクタル次元に当たります。グラフの勾配の値を伺うとフラクタル次元は約1.65であります。結果、フラクタル次元が整数でないことと、模様の位相次元より大きいということから画像内の木の枝の模様はフラクタルであるという結論に至ります。この場合、木の分枝は曲線で表すことが出来るので位相次元は１です。この場合、フラクタル次元の値が１に近いほど図形は曲線に近い性質を持っていることを表します。逆に、フラクタル次元の値が２に近いと曲線ではなく曲面に似た性質を持っていることを表します。画像内の木の模様は曲線より複雑で中心からほぼ360度に渡って分枝が存在するため、幾何学的に曲線よりは曲面に近い性質を持っています。実際、分枝の数を増やすことで模様のフラクタル次元は２に収束します。同じ位相次元を持つ二つの異なる図形をフラクタル次元という数値で比べることが出来ます。

## ご質問のある方
Eメール: kiida2@illinois.edu

