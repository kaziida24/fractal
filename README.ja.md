# フラクタル: あらゆる模様や形のフラクタル次元を推定するソフト
ステータス：プロトタイプ  
飯田一鑑  

*英語版を閲覧: [English version](README.md)*

**フラクタル**とはボックスカウント法を使用して一般的な模様や形のフラクタル次元を推定するコマンドライン専用ツールであり、[こちら](https://www.mathworks.com/matlabcentral/fileexchange/13063-boxcount/content/boxcount/html/demo.html)のMATLAB用のプログラムに基づいて作成された。このプログラムの開発に関する背景やフラクタルの数学的な概念は[プロジェクトHP](https://kaziida24.github.io/fractal)でご自由に参照してください。

## 目次
- [概要](#概要)  
	- [基本的な用途](#基本的な用途)
	- [入力データが二次元の場合](#入力データが二次元の場合)
	- [入力データが三次元の場合](#入力データが三次元の場合)
	- [プログラムの表とその役割](#プログラムの表とその役割)
- [必須ライブラリ](#必須ライブラリ)  
- [インストール法](#インストール法)  
- [使い方](#使い方)  
  - [ASCIIファイルをPCDファイルに変換](#asciiファイルをpcdファイルに変換)
  - [フラクタル解析コアプログラムを使う](#フラクタル解析コアプログラムを使う)
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


| プログラム名 | 使用言語 | 備考 | 
| :--------: | :-----: | :--: |
| fractal2d | C++ | 二次元データ（画像）専用のフラクタル解析コアプログラム |
| fractal3d | C++ | 三次元データ専用のフラクタル解析コアプログラム | 
| visualize | C++ | ポイントクラウドを視覚化するプログラム | 
| ascii2pcd | C++ | PCLが扱えるPCDファイルをASCIIポイントクラウドファイルから作成するプログラム | 
| plot_fractal.py | Python | フラクタル解析の結果をもとにフラクタル次元の推定グラフを作るプログラム | 
| plot_fractal.m | MATLAB | 上記のプログラムと同じく、MATLAB用のプログラム | 
| visualizeDataset.m | MATLAB | 特定されたポイントクラウドファイルを視覚化するMATLAB用のプログラム（[コンピュータビジョンツールボックス](http://jp.mathworks.com/products/computer-vision/index.html?s_tid=gn_loc_drop)が必要）|

<!-- ### C++ プログラム

*fractal* - フラクタル解析のコアプログラム。  
*visualize* - ポイントクラウドファイルをPCLを使って視覚化するプログラム。  
*ascii2pcd* - PCLが扱えるPCDファイルをASCIIファイルから作成するプログラム。  

### PythonとMATLAB用のプログラム

*plot_fractal.py* - フラクタル解析の結果をもとにフラクタル次元の推定グラフを作るプログラム。  
*plot_fractal.m* - *plot_fractal.py*と同じく、MATLAB用のプログラム。
*visualizeDataset.m* - 特定されたポイントクラウドファイルを視覚化するMATLAB用のプログラム。こちらを使うには
コンピュータービジョンツールボックスが必要。 -->

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

### ASCIIファイルをPCDファイルに変換

*ascii2pcd*プログラムと同時にASCIIファイル名を入力し、新しくPCDファイルが作成されます。
```bash
$ ./ascii2pcd filename.txt
```
こちらのコマンドは*filename.txt*というASCIIファイルから*filename.pcd*というPCDファイルを作成します。

### フラクタル解析コアプログラムを使う

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

## ご質問のある方
ご質問のある方は下記のプロジェクトページをご覧になってください。

プロジェクトぺージ: https://kaziida24.github.io/fractal  
Eメール: kiida2@illinois.edu

