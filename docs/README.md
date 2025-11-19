# Apt手動インストールパッケージの共有について

## セットアップ
  
以下を実行
  
```bash
$ sudo apt install pre-commit
$ pre-commit install
$ make
```
  
commitのタイミングで自動でaptのパッケージリストがpackage.txtに記録される  
強制的にmarkしたい場合は`make mark`.　

## 依存解決
  
Makefileがある場所(このプロジェクトのroot)で以下を実行  
他人が入れているpkgが自動で入る

```bash
$ make sync
```

## pre-commit無効化

`pre-commit uninstall`で無効化できる。commit時に無視して通したいときは、`git commit -m "str" --no-verify`オプションをつける。
