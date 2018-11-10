#include "PrehensionFileMenu.h"

#include <string>
#include <vector>
#include <QFileDialog>
#include <cnoid/AppConfig>
#include <cnoid/ItemTreeView>
#include <cnoid/MainWindow>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/RootItem>

#include "PlanBase.h"

using namespace std;
using namespace cnoid;

namespace {
	void invokePrehensionParamYamlFileSelectionDialog() {
		QFileDialog dialog(MainWindow::instance());
		dialog.setWindowTitle("Choose prehension parameter file");
		dialog.setFileMode(QFileDialog::ExistingFile);
		dialog.setViewMode(QFileDialog::List);
		dialog.setLabelText(QFileDialog::Accept, ("Open"));
		dialog.setLabelText(QFileDialog::Reject, ("Cancel"));

		QStringList filters;
		filters << ("Prehension parameter yaml files (*.yaml)");
		filters << ("Any files (*)");
		dialog.setNameFilters(filters);

		string currentFolder;
		if (AppConfig::archive()->read("currentFileDialogDirectory", currentFolder)) {
			dialog.setDirectory(currentFolder.c_str());
		}

		if (dialog.exec()) {
			QStringList yamlFiles = dialog.selectedFiles();
			ItemTreeView* itv = ItemTreeView::mainInstance();
			grasp::ArmFingers* parentItem = itv->selectedItem<grasp::ArmFingers>();
			if (!parentItem) {
				return;
			}
			for (int i = 0; i < yamlFiles.size(); ++i) {
				parentItem->loadPrehension(yamlFiles[i].toStdString());
			}
		}
	}
}

void cnoid::initializePrehensionFileLoader(cnoid::ExtensionManager& ext) {
	cnoid::MenuManager& mm = ext.menuManager();
	mm.setPath("/File/Open ...");
	mm.addItem("Prehension Parameter YAML")
		->sigTriggered().connect(invokePrehensionParamYamlFileSelectionDialog);
}
