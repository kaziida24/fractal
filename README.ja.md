# フラクタル解析プログラム説明書
ステータス：プロトタイプ  
飯田一鑑  

*英語版を閲覧: [English version](README.md)*

## 目次
- [概要](#概要)  
- [必須ライブラリ](#必須ライブラリ)  
- [インストール法](#インストール法)  
- [使用例](#使用例)  
- [ご質問のある方](#ご質問のある方) 

## 概要

## 必須ライブラリ  
* CMake
* PCL (ポイントクラウドライブラリ)
* Eigen
* Boost
* PythonかMATLAB (プロット作成のため）

## インストール法  
*以下のインストール法はUbuntu 14.04でしか確かめられておりません。*
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

## ご質問のある方
ご質問のある方は下記のプロジェクトページを閲覧してください。　　

プロジェクトぺージ: https://kaziida24.github.io/fractal  
Eメール: kiida2@illinois.edu

