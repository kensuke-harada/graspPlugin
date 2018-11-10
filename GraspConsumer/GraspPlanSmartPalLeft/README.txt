
GraspPlanSmartPalLeftフォルダ内の設定

(1) DirectBodyLoader.cppのコンパイルが通らなかった場合は、
右手用のDirectBodyLoader.cppをコピーする。

(2) SmartPal_standardのリンクが切れている場合は、シンボリックリンクをはる。
rm SmartPal_standard
ln -s (フォルダのパス)/SmartPal_standard SmartPal_standard 


(3) 右手で設定した把持パターンをそのまま左手使いたい場合は、
以下のコマンドを実行する。

./MirrorTransform inputfolder outputfolder

右手の把持パターンを左手用の把持パターンに変換する。
把持姿勢の掌の平面に対して、対象物が面対象であると仮定して変換する。
面対象ではない場合は誤った変換となる。

＊＊注意：outputfolderにデータが存在する場合にはファイルを上書き
＊＊注意：outputfolderは事前に作成しておく必要がある

例えば
./MirrorTransform ../GraspPlan/data ./data
