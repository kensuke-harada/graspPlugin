#include <cnoid/BodyBar>
#include <cnoid/BodyItem>

#include <QObject>
#include <QLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <QScrollBar>
#include <QFileDialog>

#include <fstream>
#include <exception>
#include <iostream>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <boost/intrusive_ptr.hpp>
#include <boost/signal.hpp>
#include <boost/signals.hpp>
#else
#include <cnoid/Signal>
#endif

#include <boost/bind.hpp>
#include <Eigen/Dense>

#include "MainView.h"
#include "FileIOAndExecutingPart.h"
#include "MotionDirectiveEditPart.h"
#include "MotionDirectiveEditLine.h"
#include "LineEditPart.h"
#include "MotionFileReader.h"
#include "MotionDirectiveInfoFormatter.h"
#include "MotionDirectiveInfoWriter.h"
#include "RobotInMotionEdit.h"
#include "MotionDirectiveInfo.h"
#include "MotionUtil.h"

#include"MotionFileControl.h"

using namespace cnoid;
using namespace std;
using namespace boost;
using namespace Eigen;

using namespace motionedit;

/**
* @brief 動作編集タブの部品全体をまとめるクラスですのコンストラクタの実装です。
*/
MainView::MainView(QWidget* parent) :QFrame(parent)
{	
        /************************************************
         * タブの部品を作成します。
 	 *************************************************/
	{	
		//ファイル入力、出力及びに実行ボタンを有する部分を作成します。
		m_FileIOAndExecutingPart = new FileIOAndExecutingPart(parent);
	        
		// フレームを作成し、
		// ファイル入力及びに実行ボタンを有する部分を収めます。
		QFrame* fileIOFrame = new QFrame(parent);
		{
			fileIOFrame->setLayout(m_FileIOAndExecutingPart);
		}
	
		// 動作指示編集部を収めます。
		m_motionDirectiveEditPart = new MotionDirectiveEditPart(parent);

		QWidget* frame = new QWidget;
		
		// フレームを作成し中に動作指示編集部を収めます。                 		
		frame->setLayout(m_motionDirectiveEditPart);	
		frame->layout()->setAlignment(Qt::AlignTop);
		
		// スクロールエリアを作成し、動作指示編集フレームを収めます。
		QScrollArea* motionDirectiveArea = new QScrollArea();
		{
			motionDirectiveArea->setWidget(frame);
			motionDirectiveArea->setWidgetResizable(true);
			motionDirectiveArea->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);	
		}

		// 作成したスクロールエリアを自身のメンバに設定します。
		m_motionDirectiveEditPartScrollArea = motionDirectiveArea;

		// ラインの増加を行う部分を作成します。
		m_LineEditPart = new LineEditPart(parent);

		QFrame* LineEditPart = new QFrame(parent);
		{
			LineEditPart->setLayout(m_LineEditPart);
		}

		// 作成した 3 つの部分をレイアウトに登録します。
		QVBoxLayout* layout = new QVBoxLayout(parent);
		{
			layout->addWidget(fileIOFrame);
			layout->addWidget(m_motionDirectiveEditPartScrollArea);
			layout->addWidget(LineEditPart);
		}

		setLayout(layout);
	}	

 	/************************************************
         * タブの部品のシグナルを連結します。
 	 *************************************************/
	{
		// ファイルセーブボタンのシグナルと自身のスロットを連結します。
		this->m_FileIOAndExecutingPart->LoadButtonClicked().connect(bind(&MainView::on_loadButton_Clicked, this));
		
		// ファイルセーブボタンのシグナルと自身のスロットを連結します。
		this->m_FileIOAndExecutingPart->SaveButtonClicked().connect(bind(&MainView::on_saveButton_Clicked, this));

		// プレイボタンのシグナルとと自身のスロットを連結します。
		this->m_FileIOAndExecutingPart->PlayButtonClicked().connect(bind(&MainView::on_playButton_Clicked, this));
		
		// 行追加ボタンのシグナルと自身のスロットを連結します。
		this->m_LineEditPart->AppendButtonClicked().connect(bind(&MainView::on_appendButton_Clicked, this));	

		// アイテムバーでロボットが変更された場合のシグナルスロットを連結します。
		BodyBar::instance()->sigCurrentBodyItemChanged().connect(bind(&MainView::on_bodyItem_Changed, this, _1));
	}
	
	{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		// 動作指示列と文字を相互に変換するためのオブジェクトを登録します。
		m_motionDirectiveTypeConverter = intrusive_ptr<MotionDirectiveTypeConverter>(new MotionDirectiveTypeConverter);

		// 手の種類と文字を相互に変換するためのオブジェクトを登録します。
		m_usingHandTypeConverter = intrusive_ptr<UsingHandTypeConverter>(new UsingHandTypeConverter);
		
		// 座標系の種類を文字を相互に変換するためのオブジェクトです。
		m_coordinateTypeConverter = intrusive_ptr<CoordinateTypeConverter> (new CoordinateTypeConverter);

                // 動作指示内容を表す文字列を解析するオブジェクトを登録します
		m_motionDirectiveWordAnalyzer = intrusive_ptr<MotionDirectiveWordAnalyzer>(
#else
		// 動作指示列と文字を相互に変換するためのオブジェクトを登録します。
		m_motionDirectiveTypeConverter = ref_ptr<MotionDirectiveTypeConverter>(new MotionDirectiveTypeConverter);

		// 手の種類と文字を相互に変換するためのオブジェクトを登録します。
		m_usingHandTypeConverter = ref_ptr<UsingHandTypeConverter>(new UsingHandTypeConverter);
		
		// 座標系の種類を文字を相互に変換するためのオブジェクトです。
		m_coordinateTypeConverter = ref_ptr<CoordinateTypeConverter> (new CoordinateTypeConverter);

                // 動作指示内容を表す文字列を解析するオブジェクトを登録します
		m_motionDirectiveWordAnalyzer = ref_ptr<MotionDirectiveWordAnalyzer>(
#endif
		new MotionDirectiveWordAnalyzer(
		m_motionDirectiveTypeConverter,
		m_usingHandTypeConverter,
		m_coordinateTypeConverter
		));
		    
		// ファイル読み込み・書き込み時の区切り文字を作成します。
	        const string delimiliterList = " ";	

		// テキストをパースするオブジェクトを作成します。     
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<TextFieldParser>textFieldParser(new TextFieldParser());
#else
		cnoid::ref_ptr<TextFieldParser>textFieldParser(new TextFieldParser());
#endif 
		{	
			// 区切り文字を登録します。	
			textFieldParser->AddDelimiter(delimiliterList);	
			// コメント行判定用トークンを登録します。
			const string comentTokenList = "#";
			textFieldParser->AddCommentToken(comentTokenList);
		}
	
		// パースされた文字列を動作指示情報型データに変換するオブジェクトを登録します。
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<ToMotionDirectiveInfoConverter> toMotionDirectiveInfoConverter(
#else
		cnoid::ref_ptr<ToMotionDirectiveInfoConverter> toMotionDirectiveInfoConverter(
#endif
			new ToMotionDirectiveInfoConverter(m_motionDirectiveWordAnalyzer));
			
                // ファイルの読み込みを行うオブジェクトを作成します。
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<MotionFileReader> motionFileReader(
#else
		cnoid::ref_ptr<MotionFileReader> motionFileReader(
#endif
			new MotionFileReader(
			textFieldParser,
			toMotionDirectiveInfoConverter
			));
		
		// 動作指示情報型データを文字列に変換するオブジェクトを登録します。
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::intrusive_ptr<MotionDirectiveInfoFormatter> formatter(
#else
		cnoid::ref_ptr<MotionDirectiveInfoFormatter> formatter(
#endif	
			new MotionDirectiveInfoFormatter(
				 m_motionDirectiveTypeConverter,
				 m_usingHandTypeConverter,
				 m_coordinateTypeConverter
				 ));
		{
	          
			formatter->SetDelimiter(delimiliterList);
		}
	        
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
                // 動作指示情報型データをストリームに書き込むオブジェクトを登録します。
		m_motionDirectiveInfoWriter = 
			intrusive_ptr<MotionDirectiveInfoWriter>(new MotionDirectiveInfoWriter(formatter));
		
		// 動作編集タブに登録するロボット情報オブジェクトを登録します。			
		SetRobot(intrusive_ptr<DefaultRobotInMotionEdit>(new DefaultRobotInMotionEdit(
		m_motionDirectiveTypeConverter, m_usingHandTypeConverter
		)));
#else
                // 動作指示情報型データをストリームに書き込むオブジェクトを登録します。
		m_motionDirectiveInfoWriter = 
			ref_ptr<MotionDirectiveInfoWriter>(new MotionDirectiveInfoWriter(formatter));
		
		// 動作編集タブに登録するロボット情報オブジェクトを登録します。			
		SetRobot(ref_ptr<DefaultRobotInMotionEdit>(new DefaultRobotInMotionEdit(
		m_motionDirectiveTypeConverter, m_usingHandTypeConverter
		)));
#endif
		
	}
	return;
}

/**
* @brief ファイル読み込みようオブジェクトを登録するメンバ関数の実装です。
*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void MainView::SetReader(intrusive_ptr<IMotionFileReader> reader)
#else
void MainView::SetReader(ref_ptr<IMotionFileReader> reader)
#endif
{
	m_motionFileReader = reader;
}

/**
* @brief ファイル書き込みようオブジェクトを登録するメンバ関数の実装です。
*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void MainView::SetWriter(intrusive_ptr<IMotionDirectiveInfoWriter> writer)
#else
void MainView::SetWriter(ref_ptr<IMotionDirectiveInfoWriter> writer)
#endif
{
	m_motionDirectiveInfoWriter = writer;
}

/**
* @brief ロボット情報を登録するメンバ関数の実装です。
*/
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
void  MainView::SetRobot(intrusive_ptr<IRobotInMotionEdit> robot)
#else
void  MainView::SetRobot(ref_ptr<IRobotInMotionEdit> robot)
#endif
{
	m_robot = robot;
	
	InitializeByRobot();

	return ;
}

/**
* @brief ロボット情報を用いて初期化を行うメンバ関数の実装です。
*/
void MainView::InitializeByRobot()
{
	m_motionDirectiveEditPart->InitializeByRobot(m_robot);
}

/**
* @brief 動作指示列のラインを追加するメンバ関数の実装です。
*/
void MainView::AddLine(int count)
{
	for (int i = 0; i < count; i++)
	{
		m_motionDirectiveEditPart->AddLine();
	}
}

/**
* @brief 与えられた動作指示情報リストをもとに
* 自身の全ての動作指示列を更新するメンバ関数の実装です。
*/
void MainView::Update(const vector<MotionDirectiveInfo> &motionDirectiveInfoList)
{
	// すべての動作指示編集列を削除します。	
	this->m_motionDirectiveEditPart->Clear();

        int lineIndex = 0;
	
	// 与えられた動作情報リストの数だけ対応した内容の動作指示編集業を作成します。
	for(vector<MotionDirectiveInfo>::const_iterator iter = motionDirectiveInfoList.begin();
            iter != motionDirectiveInfoList.end(); iter++, lineIndex++)
	{
		// 動作指示編集行を 1 行追加します。
		AddLine(1);	
		
		string motionDirectiveTypeName = 
			m_motionDirectiveTypeConverter->ConvertToString(iter->GetMotionDirectiveType());
		
		string usingHandTypeName =
			 m_usingHandTypeConverter->ConvertToString(iter->GetUsingHandType());

		string coordinateTypeName = 
			m_coordinateTypeConverter->ConvertToString(iter->GetCoordinateType());
		
		// 動作指示編集部分を動作指示情報内容を元に更新します。
		this->m_motionDirectiveEditPart->Update(
			lineIndex,
			iter->GetStartTime(),
			iter->GetEndTime(),
			usingHandTypeName + motionDirectiveTypeName,
			coordinateTypeName,
			util::Convert(iter->GetDirectiveValueVector())
			);
	}
}

/**
* @biref動作指示編集列をもとに動作指示情報のリストを作成するメンバ関数の実装です。
*/
vector<MotionDirectiveInfo> MainView::MakeMotionDirectiveInfoList()
{
	vector<MotionDirectiveInfo> motionDirectiveInfoList;
		
	int lineCount = m_motionDirectiveEditPart->GetLineCount();

	for(int lineIndex = 0; lineIndex < lineCount; lineIndex++)
	{
               /************************************************
                * 動作指示情報を作成します。
 	        *************************************************/
		MotionDirectiveInfo motionDirectiveInfo;
		{	
			motionDirectiveInfo.SetStartTime(m_motionDirectiveEditPart->GetStartTime(lineIndex));

			motionDirectiveInfo.SetEndTime(m_motionDirectiveEditPart->GetEndTime(lineIndex));

			string motionDirectiveName = m_motionDirectiveEditPart->GetMotionDirectiveTypeName(lineIndex);

			MotionDirectiveAnalyzeResult analyzeResult = m_motionDirectiveWordAnalyzer->Analyze(motionDirectiveName);	
			motionDirectiveInfo.SetUsingHandType(analyzeResult.usingHandType);
			motionDirectiveInfo.SetMotionDirectiveType(analyzeResult.motionDirectieType);

			motionDirectiveInfo.SetCoordinateType(m_coordinateTypeConverter->ConvertToCoordinateType(m_motionDirectiveEditPart->GetCoordinateTypeName(lineIndex)));

			VectorXd valueVector(m_motionDirectiveEditPart->GetMotionDirectiveValueCount(lineIndex));
		
			vector<double> valueList = m_motionDirectiveEditPart->GetMotionDirectiveValueList(lineIndex);

			for(int i = 0; i < valueList.size(); i++)
			{
				valueVector[i] = valueList[i];
			}

			motionDirectiveInfo.SetDirectiveValueVector(valueVector);
		}
		
		motionDirectiveInfoList.push_back(motionDirectiveInfo);
	}

	return motionDirectiveInfoList;
}

/**
* @brief ロードボタンが押された場合の処理です。
* ロードするファイル選択ダイアログを表示し選択されたファイルの
* 読み込み・変換を行います。失敗した場合はメッセージボックスを表示します。
*/
void  MainView::on_loadButton_Clicked()
{
	
	QString fileName = QFileDialog::getOpenFileName(
		this,
		"ファイルを開く",
		".",
		"(*.*)");

	// ファイル名が空でない場合
	if (!fileName.isEmpty())
	{
		// QString から String に文字列を変換します。
		std::string stdFileName = util::ConvertToString(fileName);

		try
		{
			// ファイル読み込みメンバを用いて読み込みを行い、得られたデータを元に画面の表示を更新します。
			MotionFileData fileData = m_motionFileReader->Read(stdFileName);
			this->Update(fileData.motionDirectiveInfoList);

			// ファイル読み込みに成功した場合のみ表示文字列を更新します。
			m_loadFileName = stdFileName;
			m_FileIOAndExecutingPart->SetLoadFileName(util::GetFileName(m_loadFileName));

			return;
		}
		// ファイル読み込み中にエラーが発生した場合
		catch(std::exception ex)
		{
			QMessageBox msg;
			msg.setText("ファイル読み込みに失敗しました。");
			msg.exec();			
			return;			
		}
		
	}
	return ;
}

/**
* @brief セーブボタンが押された場合の処理です。
* 上書き保存をする場合は選択ダイアログを表示します。
* 上書き保存しない場合は新ファイル作成ダイアログを開きます。
* 保存を行う場合はファイルオープンと動作指示情報の変換・書き込みが失敗した場合は
* メッセージボックスを表示して失敗を伝えます。
*/
void  MainView::on_saveButton_Clicked()
{
	// すでに読み込んだファイルが読み込まれている場合。
	if (!m_loadFileName.empty())
	{
		QMessageBox::StandardButton reply;
		
		// 上書きするかどうかを確認します。
		reply = QMessageBox::question(0, "ファイルの保存", "ファイルを上書き保存しますか？",
			QMessageBox::No | QMessageBox::Yes);
		
		// ファイルを上書き保存する場合
		if (reply == QMessageBox::Yes)
		{
			ofstream fs(m_loadFileName.c_str());
			
			if(!fs)
			{	// ファイルのオープンに失敗した場合
				QMessageBox msg;
				msg.setText("ファイルの書き込みに失敗しました。");
				msg.exec();				
				return;
			}
			try
			{	// ファイルがオープンできた場合
				// 書き込み用メンバを用いて動作指示編集結果をファイルに書き込みます。
				this->m_motionDirectiveInfoWriter->Write(&fs, this->MakeMotionDirectiveInfoList());
			        // ファイル書き込みが成功の場合
				// ファイルをクローズして処理を戻します。
				fs.close();

				return;
			}
			catch(const std::exception& e)
			{
				// ファイル書き込み中にエラーが発生した場合
				// ファイルをクローズします。			
				fs.close();
				QMessageBox msg;
				msg.setText("ファイル書き込みに失敗しました。");
				msg.exec();
				return;
			}
		}
	}

	// 名前をつけて保存する場合
	// QFileDialog::Options
        // ダイアログを表示します。
	QString strFName =QFileDialog::getSaveFileName(
		this,
		"名前をつけて保存",
		".",
		"(*.*dat)");

        // 文字列が空でない場合にはファイルを読み込みます。
	if (!strFName.isEmpty())
	{
		// QString から String に文字列を変換します。
		std::string stdstr = util::ConvertToString(strFName);
	
		// ファイルを開けます。
		ofstream fs(stdstr.c_str());
		if(!fs)
		{	
			QMessageBox msg;
			msg.setText("ファイル読み込みに失敗しました。");
			msg.exec();				
			return;
		}
		try{	// ファイルのオープンに成功した場合
			// ファイル読み射込みメンバを用いてファイルの内容を読み込みます。
			this->m_motionDirectiveInfoWriter->Write(&fs, this->MakeMotionDirectiveInfoList());
		
			// ファイルをクローズします。
			fs.close();

			// 書き込みに成功した場合のみ読み込みファイル名を更新します。
			m_loadFileName = stdstr;

 			m_FileIOAndExecutingPart->SetLoadFileName(util::GetFileName(m_loadFileName));

			return;
		}
		// ファイル書き込み中にエラーが発生した場合
		catch(const std::exception& e)
		{	
			fs.close();
			QMessageBox msg;
			msg.setText("ファイルの書き込みに失敗しました。");
			msg.exec();
			return;
		}			
	}
	return;
}

/**
* @brief プレイボタンがクリックされた場合に実行するスロットの実装です。
*/
void  MainView::on_playButton_Clicked()
{
 	grasp::MotionFileControl::instance()->LoadFromMotionFile(m_loadFileName);
}

/**
* @brief ライン追加ボタンがクリックされた場合に実行するスロットの実装です。
*/
void  MainView::on_appendButton_Clicked()
{
	
	this->AddLine(m_LineEditPart->GetAppendingLineCount());

	int bottomSilderPos = m_motionDirectiveEditPartScrollArea->verticalScrollBar()->maximum();
	
	m_motionDirectiveEditPartScrollArea->verticalScrollBar()->setSliderPosition(bottomSilderPos);
}

/**
* @brief いずれかの行で動作指示の種類が変更された場合に実行するスロットです。
*/
void MainView::on_motionDirectiveType_Changed(int, const QString&)
{return;}

/**
* @brief いずれかの行で座標系の種類が変更された場合に実行するスロットです。
*/
void MainView::on_coordinateType_Changed(int, const QString&)
{return;}


void  MainView::on_bodyItem_Changed(BodyItem* bodyItem)
{

}
