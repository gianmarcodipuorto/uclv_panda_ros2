
#include <QTextEdit>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QProcess>
#include <QComboBox>
#include <QStackedWidget>
#include <QCheckBox>
#include <QLineEdit>
#include <signal.h>


class PandaGui : public QWidget {
	       void onStopNode() {
		       QString nodeName = "panda_joint_velocities_bridge";
		       if (nodeProcess && nodeProcess->state() == QProcess::Running) {
			       QProcess::execute("pkill", QStringList{"-f", "panda_joint_velocities_bridge"});
				   nodeProcess->kill();
			       nodeProcess->waitForFinished(2000);
			       outputEdit->append("[INFO] " + nodeName + " stopped.");
		       } else {
			       outputEdit->append("[WARN] " + nodeName + " is not running.");
		       }
	       }
	Q_OBJECT
public:
	PandaGui(QWidget *parent = nullptr) : QWidget(parent) {
		auto *mainLayout = new QVBoxLayout(this);
		QLabel *controllerLabel = new QLabel("panda_joint_velocities_controller", this);
		mainLayout->addWidget(controllerLabel);

		// Dynamic parameter selection menu
		auto *paramMenuLayout = new QVBoxLayout();
		paramCombo = new QComboBox(this);
		paramCombo->addItems({
			"robot_ip", "safe_joint_speed_factor", "clamp", "show_details", "publish_freq",
			"flange_T_ee_translation", "flange_T_ee_rotation", "F_x_Cload", "load", "load_inertia", "critical_height", "command_timeout_ms"
		});
		paramMenuLayout->addWidget(new QLabel("Select parameter to edit:", this));
		paramMenuLayout->addWidget(paramCombo);

		// Stacked widget for parameter input
		paramStack = new QStackedWidget(this);

		// robot_ip
		QWidget *robotIpPage = new QWidget(this);
		auto *robotIpLayout = new QHBoxLayout(robotIpPage);
		robotIpEdit = new QLineEdit("10.224.20.198", this);
		robotIpLayout->addWidget(new QLabel("Robot IP:", this));
		robotIpLayout->addWidget(robotIpEdit);
		paramStack->addWidget(robotIpPage);

		// safe_joint_speed_factor
		QWidget *speedFactorPage = new QWidget(this);
		auto *speedFactorLayout = new QHBoxLayout(speedFactorPage);
		speedFactorEdit = new QLineEdit("0.9", this);
		speedFactorLayout->addWidget(new QLabel("Safe Speed Factor:", this));
		speedFactorLayout->addWidget(speedFactorEdit);
		paramStack->addWidget(speedFactorPage);

		// clamp
		QWidget *clampPage = new QWidget(this);
		auto *clampLayout = new QHBoxLayout(clampPage);
		clampCheck = new QCheckBox(this);
		clampCheck->setChecked(true);
		clampLayout->addWidget(new QLabel("Clamp:", this));
		clampLayout->addWidget(clampCheck);
		paramStack->addWidget(clampPage);

		// show_details
		QWidget *showDetailsPage = new QWidget(this);
		auto *showDetailsLayout = new QHBoxLayout(showDetailsPage);
		showDetailsCheck = new QCheckBox(this);
		showDetailsCheck->setChecked(false);
		showDetailsLayout->addWidget(new QLabel("Show Details:", this));
		showDetailsLayout->addWidget(showDetailsCheck);
		paramStack->addWidget(showDetailsPage);

		// publish_freq
		QWidget *publishFreqPage = new QWidget(this);
		auto *publishFreqLayout = new QHBoxLayout(publishFreqPage);
		publishFreqEdit = new QLineEdit("100.0", this);
		publishFreqLayout->addWidget(new QLabel("Publish Freq:", this));
		publishFreqLayout->addWidget(publishFreqEdit);
		paramStack->addWidget(publishFreqPage);

		// flange_T_ee_translation
		QWidget *flangeTransPage = new QWidget(this);
		auto *flangeTransLayout = new QHBoxLayout(flangeTransPage);
		flangeTransEdit = new QLineEdit("0.0,0.0,0.0", this);
		flangeTransLayout->addWidget(new QLabel("Flange->EE Translation:", this));
		flangeTransLayout->addWidget(flangeTransEdit);
		paramStack->addWidget(flangeTransPage);

		// flange_T_ee_rotation
		QWidget *flangeRotPage = new QWidget(this);
		auto *flangeRotLayout = new QHBoxLayout(flangeRotPage);
		flangeRotEdit = new QLineEdit("1.0,0.0,0.0,0.0", this);
		flangeRotLayout->addWidget(new QLabel("Flange->EE Rotation (quat):", this));
		flangeRotLayout->addWidget(flangeRotEdit);
		paramStack->addWidget(flangeRotPage);

		// F_x_Cload
		QWidget *fxCloadPage = new QWidget(this);
		auto *fxCloadLayout = new QHBoxLayout(fxCloadPage);
		fxCloadEdit = new QLineEdit("0.0,0.0,0.0", this);
		fxCloadLayout->addWidget(new QLabel("F_x_Cload:", this));
		fxCloadLayout->addWidget(fxCloadEdit);
		paramStack->addWidget(fxCloadPage);

		// load
		QWidget *loadPage = new QWidget(this);
		auto *loadLayout = new QHBoxLayout(loadPage);
		loadEdit = new QLineEdit("0.0", this);
		loadLayout->addWidget(new QLabel("Load:", this));
		loadLayout->addWidget(loadEdit);
		paramStack->addWidget(loadPage);

		// load_inertia
		QWidget *loadInertiaPage = new QWidget(this);
		auto *loadInertiaLayout = new QHBoxLayout(loadInertiaPage);
		loadInertiaEdit = new QLineEdit("0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0", this);
		loadInertiaLayout->addWidget(new QLabel("Load Inertia:", this));
		loadInertiaLayout->addWidget(loadInertiaEdit);
		paramStack->addWidget(loadInertiaPage);

		// critical_height
		QWidget *criticalHeightPage = new QWidget(this);
		auto *criticalHeightLayout = new QHBoxLayout(criticalHeightPage);
		criticalHeightEdit = new QLineEdit("0.1", this);
		criticalHeightLayout->addWidget(new QLabel("Critical Height:", this));
		criticalHeightLayout->addWidget(criticalHeightEdit);
		paramStack->addWidget(criticalHeightPage);

		// command_timeout_ms
		QWidget *timeoutPage = new QWidget(this);
		auto *timeoutLayout = new QHBoxLayout(timeoutPage);
		timeoutEdit = new QLineEdit("10000.0", this);
		timeoutLayout->addWidget(new QLabel("Command Timeout (ms):", this));
		timeoutLayout->addWidget(timeoutEdit);
		paramStack->addWidget(timeoutPage);

		paramMenuLayout->addWidget(paramStack);
		mainLayout->addLayout(paramMenuLayout);

		connect(paramCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), paramStack, &QStackedWidget::setCurrentIndex);

		auto *buttonLayout = new QHBoxLayout();
		startButton = new QPushButton("Start Node", this);
		configureButton = new QPushButton("Configure", this);
		activateButton = new QPushButton("Activate", this);
		dactivateButton = new QPushButton("Deactivate", this);

			   buttonLayout->addWidget(startButton);
			   buttonLayout->addWidget(configureButton);
			   buttonLayout->addWidget(activateButton);
			   buttonLayout->addWidget(dactivateButton);
			   stopButton = new QPushButton("Stop Controller", this);
			   buttonLayout->addWidget(stopButton);
		mainLayout->addLayout(buttonLayout);

		outputEdit = new QTextEdit(this);
		outputEdit->setReadOnly(true);
		mainLayout->addWidget(outputEdit);

		nodeProcess = new QProcess(this);

			   if (startButton) connect(startButton, &QPushButton::clicked, this, &PandaGui::onStartNode);
			   if (configureButton) connect(configureButton, &QPushButton::clicked, this, &PandaGui::onConfigure);
			   if (activateButton) connect(activateButton, &QPushButton::clicked, this, &PandaGui::onActivate);
			   if (dactivateButton) connect(dactivateButton, &QPushButton::clicked, this, &PandaGui::onDeactivate);
			   if (stopButton) connect(stopButton, &QPushButton::clicked, this, &PandaGui::onStopNode);

		if (nodeProcess) {
			connect(nodeProcess, &QProcess::readyReadStandardOutput, this, &PandaGui::onNodeOutput);
			connect(nodeProcess, &QProcess::readyReadStandardError, this, &PandaGui::onNodeError);
		}
	}

protected:
    void closeEvent(QCloseEvent *event) override {
        QString nodeName = "panda_joint_velocities_bridge";
        
        // 1. SIGINT come Ctrl+C (usa setsid per nuovo group, ma kill directo)
        if (nodeProcess) {
            nodeProcess->write("\x03");  // Invia Ctrl+C char al stdin (SIGINT)
            nodeProcess->waitForFinished(2000);
            
            if (nodeProcess->state() == QProcess::Running) {
                nodeProcess->terminate();
                nodeProcess->waitForFinished(1000);
            }
            
            if (nodeProcess->state() == QProcess::Running) {
                nodeProcess->kill();
            }
        }
        
        // 2. Kill robusto multi-metodo
        QProcess::execute("bash", QStringList{"-c", QString("pkill -INT -f '%1' || pkill -KILL -f '%1'").arg(nodeName)});
        QProcess::execute("bash", QStringList{"-c", QString("killall -INT '%1' || killall -KILL '%1'").arg(nodeName)});
        //QProcess::execute("bash", QStringList{"-c", "ros2 daemon stop && ros2 daemon start"});  // Reset daemon se zombie
        
        QWidget::closeEvent(event);
    }


private:
	QPushButton *startButton;
	QPushButton *configureButton;
	QPushButton *activateButton;
	QPushButton *dactivateButton;
	QPushButton *stopButton;
	QProcess *nodeProcess;
	QTextEdit *outputEdit;
	QComboBox *paramCombo;
	QStackedWidget *paramStack;
	QLineEdit *robotIpEdit;
	QLineEdit *speedFactorEdit;
	QCheckBox *clampCheck;
	QCheckBox *showDetailsCheck;
	QLineEdit *publishFreqEdit;
	QLineEdit *flangeTransEdit;
	QLineEdit *flangeRotEdit;
	QLineEdit *fxCloadEdit;
	QLineEdit *loadEdit;
	QLineEdit *loadInertiaEdit;
	QLineEdit *criticalHeightEdit;
	QLineEdit *timeoutEdit;

	void onStartNode() {
		QString nodeName = "panda_joint_velocities_bridge";
		QStringList paramArgs;
		paramArgs << QString("-p robot_ip:=%1").arg(robotIpEdit->text().trimmed());
		paramArgs << QString("-p safe_joint_speed_factor:=%1").arg(speedFactorEdit->text().trimmed());
		paramArgs << QString("-p clamp:=%1").arg(clampCheck->isChecked() ? "true" : "false");
		paramArgs << QString("-p show_details:=%1").arg(showDetailsCheck->isChecked() ? "true" : "false");
		paramArgs << QString("-p publish_freq:=%1").arg(publishFreqEdit->text().trimmed());
		// Format array parameters as [v1, v2, v3] and validate
		auto formatArray = [](const QString &input, int expectedCount, const QString &paramName, QWidget *parent) {
			QStringList vals = input.split(',', Qt::SkipEmptyParts);
			for (QString &v : vals) v = v.trimmed();
			vals.erase(std::remove_if(vals.begin(), vals.end(), [](const QString &s){ return s.isEmpty(); }), vals.end());
			if (vals.size() != expectedCount) {
				QMessageBox::warning(parent, "Parameter Error", QString("%1 expects %2 values, got %3").arg(paramName).arg(expectedCount).arg(vals.size()));
				throw std::runtime_error("Invalid parameter input");
			}
			QString arr = QString("[%1]").arg(vals.join(","));
			arr.replace(" ", ""); // Remove all spaces
			return QString("'%1'").arg(arr); // Wrap in single quotes
		};
		try {
			paramArgs << QString("-p flange_T_ee_translation:=%1").arg(formatArray(flangeTransEdit->text(), 3, "flange_T_ee_translation", this));
			paramArgs << QString("-p flange_T_ee_rotation:=%1").arg(formatArray(flangeRotEdit->text(), 4, "flange_T_ee_rotation", this));
			paramArgs << QString("-p F_x_Cload:=%1").arg(formatArray(fxCloadEdit->text(), 3, "F_x_Cload", this));
			paramArgs << QString("-p load:=%1").arg(loadEdit->text().trimmed());
			paramArgs << QString("-p load_inertia:=%1").arg(formatArray(loadInertiaEdit->text(), 9, "load_inertia", this));
		} catch (const std::exception &) {
			return;
		}
		paramArgs << QString("-p critical_height:=%1").arg(criticalHeightEdit->text().trimmed());
		paramArgs << QString("-p command_timeout_ms:=%1").arg(timeoutEdit->text().trimmed());

		QString command = QString(
			"source ~/uclv_panda_ros2/install/setup.bash && "
			"ros2 run panda_utils panda_joint_velocities_bridge --ros-args %1"
		).arg(paramArgs.join(" "));
		nodeProcess->start("bash", QStringList{"-c", command});
		if (!nodeProcess->waitForStarted(3000)) {
			QMessageBox::critical(this, "Error", "Failed to start node.");
		} else {
			outputEdit->append("[INFO] " + nodeName + " started with parameters.");
		}
	}
	void onNodeOutput() {
		outputEdit->append(nodeProcess->readAllStandardOutput());
	}

	void onNodeError() {
		outputEdit->append(nodeProcess->readAllStandardError());
	}

	void onConfigure() {
		QString nodeName = "panda_joint_velocities_bridge";
		QString command = "source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /panda_joint_velocities_bridge configure";
		QProcess::execute("bash", QStringList{"-c", command});
		QMessageBox::information(this, "Configure", nodeName + " configured.");
	
		
	}

	void onActivate() {
		QString nodeName = "panda_joint_velocities_bridge";
		QString command = "source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /panda_joint_velocities_bridge activate";
		QProcess::execute("bash", QStringList{"-c", command});
		QMessageBox::information(this, "Activate", nodeName + " activated.");
	}

	void onDeactivate() {
		QString nodeName = "panda_joint_velocities_bridge";
		QString command = "source ~/uclv_panda_ros2/install/setup.bash && ros2 lifecycle set /panda_joint_velocities_bridge deactivate";
		QProcess::execute("bash", QStringList{"-c", command});
		QMessageBox::information(this, "Deactivate", nodeName + " deactivated.");
	}
};

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);
	PandaGui gui;
	gui.setWindowTitle("Panda Controller GUI");
	gui.resize(800, 450);
	gui.show();
	return app.exec();
}

#include "panda_gui.moc"