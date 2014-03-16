#ifndef QSPAWNROBOTDIALOG_H_
#define QSPAWNROBOTDIALOG_H_

#include <QDialog>
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QIntValidator>
#include <QDoubleValidator>
#include <QPushButton>
#include <limits>


class QSpawnRobotDialog : public QDialog {
Q_OBJECT;

public:
  QSpawnRobotDialog(QWidget* parent, double defaultX, double defaultY,
      double defaultZ = 250.0, double maxFPS = 100.0) : QDialog(parent) {
    setSizeGripEnabled(false);
    setModal(true);

    uintValidator = new QIntValidator(1, std::numeric_limits<int>::max(), this);
    udoubleValidator = new QDoubleValidator(0.0, std::numeric_limits<double>::max(), 4, this);
    fpsValidator = new QDoubleValidator(0.01, maxFPS, 4, this);
    angleValidator = new QDoubleValidator(0.0, 360.0, 4, this);

    xyzLabel = new QLabel(tr("Position (x, y, z) (m): "), this);
    angleLabel = new QLabel(tr("Angle (deg): "), this);
    widthLabel = new QLabel(tr("Image Width (px): "), this);
    heightLabel = new QLabel(tr("Image Height (px): "), this);
    fpsLabel = new QLabel(tr("Image FPS (Hz): "), this);
    scaleLabel = new QLabel(tr("Scale: "), this);
    xText = new QLineEdit(QString::number(defaultX), this);
    yText = new QLineEdit(QString::number(defaultY), this);
    zText = new QLineEdit(QString::number(defaultZ), this);
    angleText = new QLineEdit(QString::number(0.0), this);
    widthText = new QLineEdit(QString::number(DEFAULT_IMAGE_WIDTH), this);
    heightText = new QLineEdit(QString::number(DEFAULT_IMAGE_HEIGHT), this);
    fpsText = new QLineEdit(QString::number(DEFAULT_IMAGE_FPS), this);
    scaleText = new QLineEdit(QString::number(1), this);
    xText->setValidator(udoubleValidator);
    yText->setValidator(udoubleValidator);
    zText->setValidator(udoubleValidator);
    angleText->setValidator(angleValidator);
    widthText->setValidator(uintValidator);
    heightText->setValidator(uintValidator);
    fpsText->setValidator(fpsValidator);
    scaleText->setValidator(udoubleValidator);

    cancelButton = new QPushButton("&Cancel", this);
    cancelButton->setDefault(false);
    cancelButton->setAutoDefault(false);
    okButton = new QPushButton("&Ok", this);
    okButton->setDefault(true);
    okButton->setAutoDefault(true);

    mainLayout = new QGridLayout(this);
    mainLayout->addWidget(xyzLabel, 0, 0);
    mainLayout->addWidget(xText, 0, 1);
    mainLayout->addWidget(yText, 0, 2);
    mainLayout->addWidget(zText, 0, 3);
    mainLayout->addWidget(angleLabel, 1, 0);
    mainLayout->addWidget(angleText, 1, 1);
    mainLayout->addWidget(scaleLabel, 1, 2);
    mainLayout->addWidget(scaleText, 1, 3);
    mainLayout->addWidget(widthLabel, 2, 0);
    mainLayout->addWidget(widthText, 2, 1);
    mainLayout->addWidget(heightLabel, 2, 2);
    mainLayout->addWidget(heightText, 2, 3);
    mainLayout->addWidget(fpsLabel, 3, 0);
    mainLayout->addWidget(fpsText, 3, 1);
    mainLayout->addWidget(cancelButton, 3, 2);
    mainLayout->addWidget(okButton, 3, 3);

    connect(cancelButton, SIGNAL(released()), this, SLOT(reject()));
    connect(okButton, SIGNAL(released()), this, SLOT(accept()));
    // DO NOT SHOW!
  };

  double getX() { return xText->text().toDouble(); };
  double getY() { return yText->text().toDouble(); };
  double getZ() { return zText->text().toDouble(); };
  double getAngle() { return angleText->text().toDouble(); };
  unsigned int getWidth() { return widthText->text().toInt(); };
  unsigned int getHeight() { return heightText->text().toInt(); };
  double getFPS() { return fpsText->text().toDouble(); };
  double getScale() { return scaleText->text().toDouble(); };

  constexpr static unsigned int DEFAULT_IMAGE_WIDTH = 320;
  constexpr static unsigned int DEFAULT_IMAGE_HEIGHT = 240;
  constexpr static double DEFAULT_IMAGE_FPS = 15.0;

protected:
  QGridLayout* mainLayout;
  QLabel* xyzLabel;
  QLabel* angleLabel;
  QLabel* widthLabel;
  QLabel* heightLabel;
  QLabel* fpsLabel;
  QLabel* scaleLabel;
  QLineEdit* xText;
  QLineEdit* yText;
  QLineEdit* zText;
  QLineEdit* angleText;
  QLineEdit* widthText;
  QLineEdit* heightText;
  QLineEdit* fpsText;
  QLineEdit* scaleText;
  QPushButton* cancelButton;
  QPushButton* okButton;
  QValidator* uintValidator;
  QValidator* angleValidator;
  QValidator* udoubleValidator;
  QValidator* fpsValidator;
};

#endif /* QSPAWNROBOTDIALOG_H_ */
