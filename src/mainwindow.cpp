#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "resources.h"
#include "scanning_3d.h"

#include <QFileSystemModel>
#include <QRegExp>
#include <QDebug>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>

using namespace RecFusion;

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::io::loadPLYFile;
using pcl::PolygonMesh;
using pcl::io::loadPolygonFilePLY;

MainWindow::MainWindow(QWidget *parent, bool use_reme)
  : QMainWindow(parent),
    ui(new Ui::MainWindow),
    scanner3d_(new RemeScanner3D()),
    add_patient_dialog_(new PatientInfoDialog),
    use_reme_(use_reme)
{
  ui->setupUi(this);
  scanning_window_ = new ScanningWindow(scanner3d_, this),
  scans_data_tree_=ui->scansDataTree;
  scans_data_tree_->set_scanning_window(scanning_window_);
  connect(scans_data_tree_, SIGNAL(VisualizeScanSignal(QString)), this, SLOT(VisualizeScanSlot(QString)));
  ui->computingDevicesComboBox->setEnabled(use_reme_);
  // ReconstructMe
  if (use_reme_) {
    ui->computingDevicesComboBox->addItems(scanner3d_->GetComputingDevices());
    // set defualt icons
    for (int i = 0; i < ui->computingDevicesComboBox->maxVisibleItems(); ++i) {
      ui->computingDevicesComboBox->setItemIcon(i, QIcon(Resources::ICON_SYNC));
    }
  }
  // RecFusion
  else {
    std::cout << "Using RecFusionSDK " << RecFusionSDK::majorVersion() << "." << RecFusionSDK::minorVersion() << std::endl;

    // Instantiate sensor objects
    m_sensor[0] = new Sensor();
    m_sensor[1] = new Sensor();
    m_sensor[2] = new Sensor();

    num_sensor_ = m_sensor[0]->deviceCount();
    QMessageBox::information(this, "Sensors connected", QString("Number of sensors connected: ") + QString::number(num_sensor_));

    for (unsigned i = 0; i < num_sensor_; ++i) {
      ok = m_sensor[i]->open(i);
      if (!ok)
      {
        QMessageBox::warning(this, "Initialization", "Couldn't open sensor num. " + QString::number(i) + ". Exiting.");
        QTimer::singleShot(0, this, SLOT(close()));
      }
      else
      {
        // Get sensor properties
        sensor_data_ = new SensorData(*m_sensor[i]);
        sensors_data_.push_back(sensor_data_);
        int w = m_sensor[i]->width();
        int h = m_sensor[i]->height();
        m_imgLabel[i]->resize(w, h);
      }
    }

    // Create message box for calibration dialog
    m_calibMessageBox = new QMessageBox(this);
    m_calibMessageBox->setIcon(QMessageBox::Information);
    m_calibMessageBox->setWindowTitle("Calibration");
    m_calibMessageBox->setText("Press OK to capture calibration frame");
    m_calibMessageBox->setDefaultButton(QMessageBox::Ok);
  }
  connect(add_patient_dialog_, SIGNAL(CreatePatientSignal(Patient)), this, SLOT(CreatePatientSlot(Patient)));
  ui->addPatientButton->setIcon(QIcon(Resources::ICON_ADD));
  ui->removePatientButton->setIcon(QIcon(Resources::ICON_REMOVE));

  // add toolbar
  QToolBar* toolbar = ui->mainToolBar;
  addToolBar(toolbar);

  QAction* a;
  a = new QAction("File", this);
  toolbar->addAction(a);

  a = new QAction("Edit", this);
  toolbar->addAction(a);

  // RecFusion
  scans_data_tree_->SetScanActionEnable(true);
}

MainWindow::~MainWindow()
{
  // Close and delete sensors
  for (unsigned i = 0; i < num_sensor_; ++i) {
    m_sensor[i]->close();
    delete m_sensor[i];
  }

  // Delete all allocated data
  delete m_timer;
  delete m_rec;
  delete sensor_data_;

  delete ui;
}

void MainWindow::on_computingDevicesComboBox_currentIndexChanged(int index)
{
  if (use_reme_) {
    int device_id = index - 1;
    if (device_id > -1) {
      ui->computingDevicesComboBox->setItemData(0, "", Qt::UserRole-1);
      if (scanner3d_->InitComputingDevice(device_id)) {
        ui->computingDevicesComboBox->setItemIcon(index, QIcon(Resources::ICON_OK));
        scans_data_tree_->SetScanActionEnable(true);
      } else {
        ui->computingDevicesComboBox->setItemIcon(index, QIcon(Resources::ICON_ERROR));
        ui->computingDevicesComboBox->setItemData(index, "", Qt::UserRole-1);
        scans_data_tree_->SetScanActionEnable(false);
      }
    }
  }
}

void MainWindow::on_addPatientButton_clicked()
{
  add_patient_dialog_->show();
}

void MainWindow::on_removePatientButton_clicked()
{
  scans_data_tree_->RemoveSelected();
}

void MainWindow::CreatePatientSlot(Patient data)
{
  scans_data_tree_->model()->Create(data);
}

void MainWindow::VisualizeScanSlot(QString scan_full_path)
{
  PointCloud<PointXYZ>::Ptr ply_cloud (new PointCloud<PointXYZ>);
  PolygonMesh mesh;
  loadPolygonFilePLY(scan_full_path.toStdString(), mesh);
  ui->scansViewer->ShowMesh(mesh);
}

void MainWindow::UpdatePatientSlot(Patient data)
{
  scans_data_tree_->model()->Update(data);
}
