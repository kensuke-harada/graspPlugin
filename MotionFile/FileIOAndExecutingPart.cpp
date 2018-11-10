#include "FileIOAndExecutingPart.h"

#include <QLabel>
#include <QSpacerItem>
#include <QFileDialog>

#include "MotionUtil.h"

using namespace cnoid;
using namespace std;
using namespace motionedit;

/**
* @ brief FileIOAndExecutingPart クラスのコンストラクタの実装です。
*/
FileIOAndExecutingPart::FileIOAndExecutingPart(QWidget* parent) :QGridLayout(parent)
{
	// 読み込んだファイル名を表示する部分を作成します。
	QLabel* fileDisplayLabel = new QLabel(tr("FileName:"));

	m_fileNameDisplay = new QLabel();
	{
		m_fileNameDisplay->setSizePolicy(
			QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)
			);
	}

	// ロードボタンを作成します。
	m_loadButton = new PushButton("Load");

	// セーブボタンを作成します。
	m_saveButton = new PushButton("Save");

	// 実行ボタンを作成します。
	m_playButton = new PushButton("Play");

	// 作成したウィジェットを自身に登録していきます。
	{
		addItem(new QSpacerItem(40, 20), 1, 0);
	
		addWidget(fileDisplayLabel, 0, 1);

		addWidget(m_fileNameDisplay, 1, 1);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 2);

		addWidget(m_loadButton, 1, 3);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 4);

		addWidget(m_saveButton, 1, 5);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Preferred), 1, 6);

		addWidget(m_playButton, 1, 7);

		addItem(new QSpacerItem(40, 20, QSizePolicy::Expanding), 1, 8);
	}
	return;
}

/**
* @brief 読み込んだファイル名を元に表示部を更新するメンバ関数の実装です。
*/
void FileIOAndExecutingPart::SetLoadFileName(std::string loadFileName)
{
	QString qFileName = util::ConvertToQString(loadFileName);

	m_fileNameDisplay->setText(qFileName);


	return;
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
/**
* @brief ロードボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<boost::signal<void(bool)> > FileIOAndExecutingPart::LoadButtonClicked()
{
	return m_loadButton->sigClicked();
}

/**
* @brief セーブボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<boost::signal<void(bool)> > FileIOAndExecutingPart::SaveButtonClicked()
{
	return m_saveButton->sigClicked();
}

/**
* @brief プレイボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<boost::signal<void(bool)> > FileIOAndExecutingPart::PlayButtonClicked()
{
	return m_playButton->sigClicked();
}
#elif defined(CNOID_15)
/**
* @brief ロードボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<void(bool)> FileIOAndExecutingPart::LoadButtonClicked()
{
	return m_loadButton->sigClicked();
}

/**
* @brief セーブボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<void(bool)> FileIOAndExecutingPart::SaveButtonClicked()
{
	return m_saveButton->sigClicked();
}

/**
* @brief プレイボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<void(bool)> FileIOAndExecutingPart::PlayButtonClicked()
{
	return m_playButton->sigClicked();
}
#else
/**
* @brief ロードボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<void()> FileIOAndExecutingPart::LoadButtonClicked()
{
	return m_loadButton->sigClicked();
}

/**
* @brief セーブボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<void()> FileIOAndExecutingPart::SaveButtonClicked()
{
	return m_saveButton->sigClicked();
}

/**
* @brief プレイボタンが押された場合に発生するシグナルを返すメンバ関数の実装です。
*/
SignalProxy<void()> FileIOAndExecutingPart::PlayButtonClicked()
{
	return m_playButton->sigClicked();
}
#endif


