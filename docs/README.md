# Apt手動インストールパッケージの共有について

## pre-commitの導入
  
以下を実行
  
```bash
$ sudo apt install pre-commit
$ pre-commit install
```
  
commitのタイミングで自動でaptのパッケージリストがpackage.txtに記録される  
強制的にmarkしたい場合は`make mark`

## 依存解決
  
Makefileがある場所(このプロジェクトのroot)で以下を実行  
他人が入れているpkgが自動で入る

```bash
$ make sync
```
