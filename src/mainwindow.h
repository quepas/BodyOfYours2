#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "data/patient.h"
#include "patientinfodialog.h"
#include "scans_data_tree.h"
#include "scanner_3d.h"
#include "scanningwindow.h"
#include "reme_scanner_3d.h"
#include "sensor_data.h"

#include <QMainWindow>
#include <QModelIndex>

#include <RecFusion.h>

class QLabel;
class QMessageBox;
class QTimer;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0, bool use_reme = false);
  ~MainWindow();

private slots:  
  void on_computingDevicesComboBox_currentIndexChanged(int index);
  void on_addPatientButton_clicked();
  void on_removePatientButton_clicked();

  void CreatePatientSlot(Patient data);
  void UpdatePatientSlot(Patient data);
  void VisualizeScanSlot(QString scan_full_path);

private:
  Ui::MainWindow *ui;
  ScanningWindow *scanning_window_;
  PatientInfoDialog *add_patient_dialog_;
  ScansDataTree* scans_data_tree_;
  Scanner3D* scanner3d_;
  bool use_reme_;

  QLabel* m_imgLabel[3];
  QLabel* m_recLabel[3];
  QMessageBox* m_calibMessageBox;
  QTimer* m_timer;

  RecFusion::Sensor* m_sensor[3];
  RecFusion::Mat4 m_sensorT[3];

  SensorData* sensor_data_;
  std::vector<SensorData*> sensors_data_;
  unsigned num_sensor_;
};

#endif // MAINWINDOW_H
