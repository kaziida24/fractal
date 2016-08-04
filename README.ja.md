# フラクタル解析プログラム説明書
ステータス：プロトタイプ  
飯田一鑑  

*英語版を閲覧: [English version](README.md)*

![alt text](https://raw.githubusercontent.com/kaziida24/fractal/master/figures/program_overview_ja.png "プログラムの役割")

## 目次
- [概要](#概要)  
  - [C++ プログラム](#c-%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%A0)
  - [PythonとMATLAB用のプログラム](#pythonとmatlab用のプログラム)
- [必須ライブラリ](#必須ライブラリ)  
- [インストール法](#インストール法)  
- [使用例](#使用例)  
  - [ASCIIファイルをPCDファイルに変換](#asciiファイルをpcdファイルに変換)
  - [フラクタル解析コアプログラムを使う](#フラクタル解析コアプログラムを使う)
  - [PCLを使用してポイントクラウドファイルを視覚化](#pclを使用してポイントクラウドファイルを視覚化)
  - [ヘルプメニューを引き出す](#ヘルプメニューを引き出す)
- [ご質問のある方](#ご質問のある方) 

## 概要
こちらのプロジェクトにはフラクタル解析と形状の視覚化を目的としたプログラムを含めております。

### C++ プログラム

*fractal* - フラクタル解析のコアプログラム。  
*visualize* - ポイントクラウドファイルをPCLを使って視覚化するプログラム。  
*ascii2pcd* - PCLが扱えるPCDファイルをASCIIファイルから作成するプログラム。  

### PythonとMATLAB用のプログラム

*plot_fractal.py* - フラクタル解析の結果をもとにフラクタル次元の推定グラフを作るプログラム。  
*plot_fractal.m* - *plot_fractal.py*と同じく、MATLAB用のプログラム。
*visualizeDataset.m* - 特定されたポイントクラウドファイルを視覚化するMATLAB用のプログラム。こちらを使うには
コンピュータービジョンツールボックスが必要。

## 必須ライブラリ  
* CMake
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

## 使用例  

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

## ご質問のある方
ご質問のある方は下記のプロジェクトページをご覧になってください。

プロジェクトぺージ: https://kaziida24.github.io/fractal  
Eメール: kiida2@illinois.edu

