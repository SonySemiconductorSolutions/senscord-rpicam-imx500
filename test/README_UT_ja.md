# senscord-rpicam-imx500 ユニットテストドキュメント

このドキュメントでは、senscord-rpicam-imx500コンポーネントのユニットテストフレームワークと手順について説明します。

## 概要

ユニットテストフレームワークは、senscord-rpicam-imx500プロジェクトのすべてのコンポーネントに対して包括的なテストカバレッジを提供するように設計されています。テストフレームワークとしてGoogle Test（GTest）とGoogle Mock（GMock）を使用し、ビルド管理にはCMake、カバレッジレポートにはlcovを使用しています。

## GitHub Actionsワークフロー

### run_unit_test.yml

GitHub Actionsワークフロー（`run_unit_test.yml`）は、プルリクエストと手動トリガーに対して自動化されたユニットテストを提供します。

#### 機能

- **自動PRテスト**: プルリクエストで自動的にテストを実行
- **スマート実行**: ドキュメントやパッチファイルのみが変更された場合はテストをスキップ
- **手動トリガー**: `workflow_dispatch`を使用して手動でトリガー可能
- **再利用可能ワークフロー**: `workflow_call`を使用して他のワークフローから呼び出し可能
- **カバレッジレポート**: HTMLカバレッジレポートを生成・アップロード
- **アーティファクトアップロード**: テストログとカバレッジレポートをアーティファクトとして保存

#### ワークフローステップ

1. **必要性チェック**: 変更されたファイルに基づいてテストを実行する必要があるかを判定
2. **環境セットアップ**: `env_setup.sh`を使用して必要な依存関係をインストール
3. **テスト実行**: `run_unit_test.sh`を使用してユニットテストを実行
4. **カバレッジ収集**: lcovでカバレッジレポートを生成
5. **アーティファクトアップロード**: レビュー用にログとカバレッジレポートをアップロード

## シェルスクリプト

### env_setup.sh

ユニットテストのためにシステムを準備する環境セットアップスクリプトです。

#### 目的

- 必須ビルドツール（cmake、gcc、g++など）のインストール
- テストフレームワーク（Google Test、Google Mock）のインストール
- カバレッジツール（lcov、gcovr）のインストール
- テスト用システム環境の設定

#### 使い方

```bash
sudo bash env_setup.sh
```

#### インストールするもの

- **ビルドツール**: build-essential、cmake、make、gcc、g++、pkg-config
- **テストライブラリ**: libgtest-dev、libgmock-dev、googletest
- **カバレッジツール**: lcov、gcovr
- **スレッドサポート**: libpthread-stubs0-dev
- **開発ツール**: git、curl、wget、unzip、zip

#### 環境検証

スクリプトには、すべてのツールが適切にインストールされてアクセス可能であることを確認する検証ステップが含まれています。

### exec_unit_test.sh

個別コンポーネントのビルド、テスト、カバレッジデータ収集を行うスクリプトです。

#### 使い方

```bash
./exec_unit_test.sh [build|test] <directory_path>
./exec_unit_test.sh collect [directory_path]
```

#### コマンド

- **build**: 指定されたディレクトリのユニットテストをビルド
- **test**: 指定されたディレクトリのユニットテストを実行
- **collect**: 指定されたディレクトリまたは全体のカバレッジデータを収集してHTMLレポートを生成

#### 引数

- **directory_path**: テストディレクトリのパス（例：sample、public/component/libcamera_imageなど）
- **collectコマンドの場合**: directory_pathは省略可能（省略時は統合カバレッジを収集）

#### 例

```bash
./exec_unit_test.sh build sample               # sampleテストをビルド
./exec_unit_test.sh test sample                # sampleテストを実行
./exec_unit_test.sh collect sample             # sampleのカバレッジを収集
./exec_unit_test.sh collect                    # 全コンポーネントの統合カバレッジを収集
```

#### 機能

- ターゲットディレクトリとCMakeLists.txtの存在を検証
- ビルドディレクトリを自動作成
- DebugコンフィグレーションでCMakeを使用
- CTestでテストを実行
- lcovでカバレッジレポートを生成
- システムヘッダーとテストファイルをカバレッジから除外

### run_unit_test.sh

すべてのユニットテストコンポーネントを実行し、統合カバレッジレポートを生成するマスタースクリプトです。

#### 使い方

```bash
./run_unit_test.sh
```

#### 動作内容

1. **全コンポーネントビルド**: 設定されたすべてのユニットテストコンポーネントをビルド
2. **全テスト実行**: すべてのコンポーネントのテストを実行
3. **統合カバレッジ収集**: すべてのコンポーネントから統合カバレッジデータを収集してHTMLレポートを生成

#### コンポーネント設定

コンポーネントはスクリプト内の`COMPONENTS`配列で定義されます：

```bash
COMPONENTS=(
    "sample"
    # 他のコンポーネントを作成時にここに追加
    # "public/component/libcamera_image"
    # "public/component/inference_property_converter"
)
```

#### 出力

- `<component>/coverage/`に個別コンポーネントカバレッジレポート
- `test/coverage/html/`に統合カバレッジレポート
- CI統合用の統合カバレッジ情報ファイル

## テストサンプルファイル

`test/sample/`ディレクトリには、テストフレームワークの使用方法を示すサンプル実装が含まれています。

### ファイル構造

#### ソースファイル

- **sample_src.c**: テスト対象の関数の実装
  - `SampleFunc()`: 外部モジュールに依存する基本関数
  - `SampleMallocFunc()`: メモリ割り当てテストを示す関数
  - `SampleFreeFunc()`: メモリ解放テストを示す関数

- **sample_src.h**: テスト関数を宣言するヘッダーファイル

- **sample_sub.h**: 外部依存関係のヘッダー（テストでモック化）

#### モックファイル

- **sample_mock.cc**: `SampleSub`関数のモック実装
  - モックインスタンス管理にシングルトンパターンを使用
  - 期待値をクリアする`Reset()`メソッドを提供

- **sample_mock.h**: モックインターフェースを宣言するヘッダー
  - GMockフレームワークを拡張
  - `SampleSub`関数の`MOCK_METHOD`を定義

#### テストファイル

- **sample_gtest.cc**: Google Testの実装
  - GMockを使用した基本的なモック化を実演
  - `MallocMock`を使用したメモリ割り当て/解放テストを表示
  - モック管理のためのテスト環境セットアップを含む
  - 様々なテストシナリオ（成功、失敗、バイパスモード）を含む

#### ビルド設定

- **CMakeLists.txt**: サンプルテストのCMake設定
  - C/C++標準とコンパイラフラグを設定
  - カバレッジフラグ（`--coverage -O0 -g`）を設定
  - 必要なライブラリ（GTest、GMock、malloc_mock）をリンク
  - `UNIT_TEST`プリプロセッサマクロを定義

### モックライブラリ

#### MallocMock（`test/mock/malloc/`）

- **malloc_mock.h/cc**: malloc/free関数のモック実装
- **CMakeLists.txt**: リンカーラップオプション付きビルド設定
- **機能**:
  - 実際のmalloc/freeを使用するバイパスモード
  - 制御されたテスト用のモックモード
  - スレッドセーフなシングルトンパターン
  - リンカーラッピング（`-Wl,--wrap=malloc -Wl,--wrap=free`）

## コンポーネントのテスト作成方法

### 新しいコンポーネントテストの追加

新しいコンポーネント（例：`libcamera_image`）のユニットテストを追加するには：

#### 1. テストディレクトリ構造の作成

```
test/
├── public/
│   └── component/
│       └── libcamera_image/
│           ├── core/
│           │   ├── CMakeLists.txt
│           │   └── libcamera_image_test.cc
│           └── mock/
│               ├── CMakeLists.txt
│               ├── libcamera_image_mock.h
│               └── libcamera_image_mock.cc
```

#### 2. CMakeLists.txtの設定

**コアテストCMakeLists.txt** (`test/public/component/libcamera_image/core/CMakeLists.txt`):
```cmake
cmake_minimum_required(VERSION 3.16)
project(LibcameraImageUnitTest)

# 標準とフラグの設定
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -O0 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -g")

# パッケージの検索
find_package(GTest REQUIRED)

# モックサブディレクトリを追加
add_subdirectory(../mock ${CMAKE_CURRENT_BINARY_DIR}/mock)

# 実際のコンポーネントからのソースファイル
set(SRC_FILES
    ../../../../../public/component/libcamera_image/src/libcamera_image_stream_source.cpp
    # 必要に応じて他のソースファイルを追加
)

# テスト実行ファイルの作成
add_executable(libcamera_image_unit_test 
    ${SRC_FILES}
    libcamera_image_test.cc
)

# インクルードディレクトリ
target_include_directories(libcamera_image_unit_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../../public/component/libcamera_image/include
    ../mock
    # 他のインクルードパスを追加
)

# プリプロセッサマクロの定義
target_compile_definitions(libcamera_image_unit_test PRIVATE UNIT_TEST=1)

# ライブラリのリンク
target_link_libraries(libcamera_image_unit_test
    libcamera_image_mock
    GTest::gtest
    GTest::gmock
    Threads::Threads
    gcov
)

# CTestにテストを追加
add_test(NAME LibcameraImageUnitTest COMMAND libcamera_image_unit_test)
```

**モックCMakeLists.txt** (`test/public/component/libcamera_image/mock/CMakeLists.txt`):
```cmake
# libcamera_image用モックライブラリ
add_library(libcamera_image_mock STATIC
    libcamera_image_mock.cc
)

# インクルードディレクトリ
target_include_directories(libcamera_image_mock PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../../../../public/component/libcamera_image/include
)

# 必要なパッケージの検索
find_package(GTest REQUIRED)

# ライブラリのリンク
target_link_libraries(libcamera_image_mock PUBLIC
    GTest::gmock
)
```

#### 3. コンポーネントリストの更新

`run_unit_test.sh`の`COMPONENTS`配列に新しいコンポーネントを追加：

```bash
COMPONENTS=(
    "sample"
    "public/component/libcamera_image"  # フルパスで新しいコンポーネントをここに追加
    # "public/component/inference_property_converter"
)
```

#### 4. モックインターフェースの作成

```cpp
// libcamera_image_mock.h
#ifndef SENSCORD_RPICAM_IMX500_TEST_LIBCAMERA_IMAGE_MOCK_H_
#define SENSCORD_RPICAM_IMX500_TEST_LIBCAMERA_IMAGE_MOCK_H_

#include <gmock/gmock.h>

class LibcameraImageMock {
 public:
  static LibcameraImageMock& GetInstance();
  
  // 依存関係のモックメソッド
  MOCK_METHOD(int, dependency_function, (int param), (const));
  
  void Reset() {
    testing::Mock::VerifyAndClearExpectations(this);
  }

 private:
  LibcameraImageMock() = default;
  LibcameraImageMock(const LibcameraImageMock&) = delete;
  LibcameraImageMock& operator=(const LibcameraImageMock&) = delete;
};

#endif
```

#### 5. テストケースの実装

```cpp
// libcamera_image_test.cc
#include <gtest/gtest.h>
#include "libcamera_image_mock.h"

extern "C" {
#include "senscord/libcamera_image/libcamera_image_types.h"
}

class LibcameraImageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 各テスト前のセットアップ
  }
  
  void TearDown() override {
    // 各テスト後のクリーンアップ
    mock.Reset();
  }
  
  LibcameraImageMock& mock = LibcameraImageMock::GetInstance();
};

TEST_F(LibcameraImageTest, BasicFunctionality) {
  // モック期待値の設定
  EXPECT_CALL(mock, dependency_function(testing::_))
      .WillOnce(testing::Return(42));
  
  // テスト実行
  int result = libcamera_image_function();
  
  // 結果の検証
  EXPECT_EQ(result, expected_value);
}
```

### ベストプラクティス

1. **依存関係の分離**: すべての外部依存関係をモック化
2. **説明的なテスト名**: テストの目的を明確にする
3. **エッジケースのテスト**: 境界条件とエラーケースを含む
4. **モック状態の維持**: テスト間でモックをリセット
5. **カバレッジ目標**: 意味のあるテストを確保しながら高いコードカバレッジを目指す
6. **コンポーネント分離**: 特定のコンポーネント機能に焦点を当てたテストを維持

### テストの実行

#### 個別コンポーネント

```bash
cd test/scripts
./exec_unit_test.sh build public/component/libcamera_image
./exec_unit_test.sh test public/component/libcamera_image
./exec_unit_test.sh collect public/component/libcamera_image
```

#### 全コンポーネント

```bash
cd test/scripts
./run_unit_test.sh
```

統合カバレッジレポートは`test/coverage/html/index.html`で利用できます。
