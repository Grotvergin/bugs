/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *imageViewer;
    QFrame *optionsViewFrame;
    QCheckBox *zAxisScaleLabel;
    QCheckBox *dimensionScaleLabel;
    QCheckBox *xAxisRotateLabel;
    QLabel *label_2;
    QDoubleSpinBox *heightScale;
    QDoubleSpinBox *sizesScale;
    QDoubleSpinBox *xAxisRotateAngle;
    QLabel *label_3;
    QFrame *frame_4;
    QFrame *frame_5;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QDoubleSpinBox *widthValue;
    QDoubleSpinBox *lengthValue;
    QDoubleSpinBox *heightValue;
    QLabel *label_7;
    QFrame *frame;
    QSlider *smoothingSlider;
    QDoubleSpinBox *smoothingSpinBox;
    QCheckBox *smoothingOptionCheckbox;
    QCheckBox *sharpnessOptionCheckbox;
    QDoubleSpinBox *sharpnessSpinBox;
    QSlider *sharpnessSlider;
    QCheckBox *colorInverseOptionCheckbox;
    QCheckBox *grayscaleOptionCheckbox;
    QPushButton *convertStartButton;
    QProgressBar *convertStatusProgressBar;
    QFrame *frame_2;
    QLineEdit *outputFilenameEdit;
    QLineEdit *outputFolderPathEdit;
    QPushButton *outputFolderSelect;
    QPushButton *selectImageButton;
    QPushButton *textureImageSelect;
    QLineEdit *selectedTexturePath;
    QFrame *frame_3;
    QRadioButton *pngToDaeSelect;
    QRadioButton *pngToStlSelect;
    QRadioButton *stlToDaeSelect;
    QLabel *label;
    QLabel *textureViewer;
    QLabel *label_8;
    QLabel *label_9;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(724, 612);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        imageViewer = new QLabel(centralWidget);
        imageViewer->setObjectName(QString::fromUtf8("imageViewer"));
        imageViewer->setGeometry(QRect(440, 30, 271, 231));
        imageViewer->setFrameShape(QFrame::Box);
        imageViewer->setFrameShadow(QFrame::Plain);
        imageViewer->setAlignment(Qt::AlignCenter);
        optionsViewFrame = new QFrame(centralWidget);
        optionsViewFrame->setObjectName(QString::fromUtf8("optionsViewFrame"));
        optionsViewFrame->setGeometry(QRect(10, 240, 421, 271));
        optionsViewFrame->setFrameShape(QFrame::StyledPanel);
        optionsViewFrame->setFrameShadow(QFrame::Raised);
        zAxisScaleLabel = new QCheckBox(optionsViewFrame);
        zAxisScaleLabel->setObjectName(QString::fromUtf8("zAxisScaleLabel"));
        zAxisScaleLabel->setGeometry(QRect(10, 30, 131, 23));
        QFont font;
        font.setPointSize(10);
        zAxisScaleLabel->setFont(font);
        zAxisScaleLabel->setChecked(false);
        dimensionScaleLabel = new QCheckBox(optionsViewFrame);
        dimensionScaleLabel->setObjectName(QString::fromUtf8("dimensionScaleLabel"));
        dimensionScaleLabel->setGeometry(QRect(10, 60, 131, 23));
        dimensionScaleLabel->setFont(font);
        xAxisRotateLabel = new QCheckBox(optionsViewFrame);
        xAxisRotateLabel->setObjectName(QString::fromUtf8("xAxisRotateLabel"));
        xAxisRotateLabel->setGeometry(QRect(10, 90, 131, 23));
        xAxisRotateLabel->setFont(font);
        label_2 = new QLabel(optionsViewFrame);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 0, 121, 17));
        heightScale = new QDoubleSpinBox(optionsViewFrame);
        heightScale->setObjectName(QString::fromUtf8("heightScale"));
        heightScale->setGeometry(QRect(150, 30, 69, 26));
        heightScale->setFont(font);
        heightScale->setSingleStep(0.100000000000000);
        heightScale->setValue(0.250000000000000);
        sizesScale = new QDoubleSpinBox(optionsViewFrame);
        sizesScale->setObjectName(QString::fromUtf8("sizesScale"));
        sizesScale->setGeometry(QRect(150, 60, 69, 26));
        sizesScale->setFont(font);
        sizesScale->setSingleStep(0.100000000000000);
        sizesScale->setValue(0.050000000000000);
        xAxisRotateAngle = new QDoubleSpinBox(optionsViewFrame);
        xAxisRotateAngle->setObjectName(QString::fromUtf8("xAxisRotateAngle"));
        xAxisRotateAngle->setGeometry(QRect(150, 90, 69, 26));
        xAxisRotateAngle->setFont(font);
        xAxisRotateAngle->setMinimum(-360.000000000000000);
        xAxisRotateAngle->setMaximum(360.000000000000000);
        xAxisRotateAngle->setSingleStep(0.100000000000000);
        xAxisRotateAngle->setValue(-90.000000000000000);
        label_3 = new QLabel(optionsViewFrame);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(240, 0, 171, 20));
        frame_4 = new QFrame(optionsViewFrame);
        frame_4->setObjectName(QString::fromUtf8("frame_4"));
        frame_4->setGeometry(QRect(0, 0, 231, 121));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Raised);
        frame_5 = new QFrame(optionsViewFrame);
        frame_5->setObjectName(QString::fromUtf8("frame_5"));
        frame_5->setGeometry(QRect(230, -1, 191, 121));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Raised);
        label_4 = new QLabel(frame_5);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 30, 67, 17));
        label_4->setFont(font);
        label_5 = new QLabel(frame_5);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 60, 67, 17));
        label_5->setFont(font);
        label_6 = new QLabel(frame_5);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 90, 67, 17));
        label_6->setFont(font);
        widthValue = new QDoubleSpinBox(frame_5);
        widthValue->setObjectName(QString::fromUtf8("widthValue"));
        widthValue->setGeometry(QRect(100, 30, 69, 26));
        widthValue->setFont(font);
        widthValue->setReadOnly(true);
        widthValue->setMaximum(999.990000000000009);
        widthValue->setSingleStep(0.100000000000000);
        lengthValue = new QDoubleSpinBox(frame_5);
        lengthValue->setObjectName(QString::fromUtf8("lengthValue"));
        lengthValue->setGeometry(QRect(100, 60, 69, 26));
        lengthValue->setFont(font);
        lengthValue->setReadOnly(true);
        lengthValue->setMaximum(999.990000000000009);
        lengthValue->setSingleStep(0.100000000000000);
        heightValue = new QDoubleSpinBox(frame_5);
        heightValue->setObjectName(QString::fromUtf8("heightValue"));
        heightValue->setGeometry(QRect(100, 90, 69, 26));
        heightValue->setFont(font);
        heightValue->setReadOnly(true);
        heightValue->setMaximum(999.990000000000009);
        heightValue->setSingleStep(0.100000000000000);
        label_7 = new QLabel(optionsViewFrame);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 120, 181, 17));
        frame = new QFrame(optionsViewFrame);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(10, 140, 271, 101));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        smoothingSlider = new QSlider(frame);
        smoothingSlider->setObjectName(QString::fromUtf8("smoothingSlider"));
        smoothingSlider->setGeometry(QRect(10, 70, 121, 16));
        smoothingSlider->setMaximum(50);
        smoothingSlider->setSingleStep(1);
        smoothingSlider->setSliderPosition(0);
        smoothingSlider->setOrientation(Qt::Horizontal);
        smoothingSpinBox = new QDoubleSpinBox(frame);
        smoothingSpinBox->setObjectName(QString::fromUtf8("smoothingSpinBox"));
        smoothingSpinBox->setGeometry(QRect(20, 40, 61, 26));
        smoothingSpinBox->setReadOnly(true);
        smoothingSpinBox->setDecimals(1);
        smoothingSpinBox->setMaximum(50.000000000000000);
        smoothingSpinBox->setSingleStep(0.100000000000000);
        smoothingSpinBox->setStepType(QAbstractSpinBox::DefaultStepType);
        smoothingOptionCheckbox = new QCheckBox(frame);
        smoothingOptionCheckbox->setObjectName(QString::fromUtf8("smoothingOptionCheckbox"));
        smoothingOptionCheckbox->setGeometry(QRect(10, 10, 101, 23));
        smoothingOptionCheckbox->setFont(font);
        sharpnessOptionCheckbox = new QCheckBox(frame);
        sharpnessOptionCheckbox->setObjectName(QString::fromUtf8("sharpnessOptionCheckbox"));
        sharpnessOptionCheckbox->setGeometry(QRect(140, 10, 82, 23));
        sharpnessOptionCheckbox->setFont(font);
        sharpnessSpinBox = new QDoubleSpinBox(frame);
        sharpnessSpinBox->setObjectName(QString::fromUtf8("sharpnessSpinBox"));
        sharpnessSpinBox->setGeometry(QRect(150, 40, 62, 26));
        sharpnessSpinBox->setReadOnly(true);
        sharpnessSpinBox->setDecimals(1);
        sharpnessSpinBox->setMaximum(50.000000000000000);
        sharpnessSpinBox->setSingleStep(0.100000000000000);
        sharpnessSpinBox->setStepType(QAbstractSpinBox::DefaultStepType);
        sharpnessSlider = new QSlider(frame);
        sharpnessSlider->setObjectName(QString::fromUtf8("sharpnessSlider"));
        sharpnessSlider->setGeometry(QRect(140, 70, 121, 20));
        sharpnessSlider->setMaximum(50);
        sharpnessSlider->setOrientation(Qt::Horizontal);
        colorInverseOptionCheckbox = new QCheckBox(optionsViewFrame);
        colorInverseOptionCheckbox->setObjectName(QString::fromUtf8("colorInverseOptionCheckbox"));
        colorInverseOptionCheckbox->setGeometry(QRect(300, 140, 111, 23));
        colorInverseOptionCheckbox->setFont(font);
        grayscaleOptionCheckbox = new QCheckBox(optionsViewFrame);
        grayscaleOptionCheckbox->setObjectName(QString::fromUtf8("grayscaleOptionCheckbox"));
        grayscaleOptionCheckbox->setGeometry(QRect(300, 170, 101, 23));
        grayscaleOptionCheckbox->setFont(font);
        frame_5->raise();
        frame_4->raise();
        zAxisScaleLabel->raise();
        dimensionScaleLabel->raise();
        xAxisRotateLabel->raise();
        label_2->raise();
        heightScale->raise();
        sizesScale->raise();
        xAxisRotateAngle->raise();
        label_3->raise();
        label_7->raise();
        frame->raise();
        colorInverseOptionCheckbox->raise();
        grayscaleOptionCheckbox->raise();
        convertStartButton = new QPushButton(centralWidget);
        convertStartButton->setObjectName(QString::fromUtf8("convertStartButton"));
        convertStartButton->setGeometry(QRect(180, 520, 80, 25));
        convertStartButton->setFont(font);
        convertStatusProgressBar = new QProgressBar(centralWidget);
        convertStatusProgressBar->setObjectName(QString::fromUtf8("convertStatusProgressBar"));
        convertStatusProgressBar->setGeometry(QRect(100, 550, 251, 23));
        convertStatusProgressBar->setMaximum(1);
        convertStatusProgressBar->setValue(-1);
        frame_2 = new QFrame(centralWidget);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(10, 10, 421, 131));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        outputFilenameEdit = new QLineEdit(frame_2);
        outputFilenameEdit->setObjectName(QString::fromUtf8("outputFilenameEdit"));
        outputFilenameEdit->setGeometry(QRect(150, 10, 261, 25));
        outputFilenameEdit->setFont(font);
        outputFolderPathEdit = new QLineEdit(frame_2);
        outputFolderPathEdit->setObjectName(QString::fromUtf8("outputFolderPathEdit"));
        outputFolderPathEdit->setGeometry(QRect(150, 50, 261, 25));
        outputFolderPathEdit->setFont(font);
        outputFolderSelect = new QPushButton(frame_2);
        outputFolderSelect->setObjectName(QString::fromUtf8("outputFolderSelect"));
        outputFolderSelect->setGeometry(QRect(10, 50, 131, 25));
        outputFolderSelect->setFont(font);
        selectImageButton = new QPushButton(frame_2);
        selectImageButton->setObjectName(QString::fromUtf8("selectImageButton"));
        selectImageButton->setGeometry(QRect(10, 10, 131, 25));
        selectImageButton->setFont(font);
        textureImageSelect = new QPushButton(frame_2);
        textureImageSelect->setObjectName(QString::fromUtf8("textureImageSelect"));
        textureImageSelect->setGeometry(QRect(10, 90, 131, 25));
        textureImageSelect->setFont(font);
        selectedTexturePath = new QLineEdit(frame_2);
        selectedTexturePath->setObjectName(QString::fromUtf8("selectedTexturePath"));
        selectedTexturePath->setGeometry(QRect(150, 90, 261, 25));
        selectedTexturePath->setFont(font);
        selectedTexturePath->setReadOnly(true);
        frame_3 = new QFrame(centralWidget);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        frame_3->setGeometry(QRect(10, 160, 421, 61));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        pngToDaeSelect = new QRadioButton(frame_3);
        pngToDaeSelect->setObjectName(QString::fromUtf8("pngToDaeSelect"));
        pngToDaeSelect->setGeometry(QRect(10, 30, 101, 23));
        pngToDaeSelect->setFont(font);
        pngToDaeSelect->setChecked(true);
        pngToStlSelect = new QRadioButton(frame_3);
        pngToStlSelect->setObjectName(QString::fromUtf8("pngToStlSelect"));
        pngToStlSelect->setGeometry(QRect(130, 30, 96, 23));
        pngToStlSelect->setFont(font);
        stlToDaeSelect = new QRadioButton(frame_3);
        stlToDaeSelect->setObjectName(QString::fromUtf8("stlToDaeSelect"));
        stlToDaeSelect->setGeometry(QRect(240, 30, 96, 23));
        stlToDaeSelect->setFont(font);
        label = new QLabel(frame_3);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 0, 121, 17));
        textureViewer = new QLabel(centralWidget);
        textureViewer->setObjectName(QString::fromUtf8("textureViewer"));
        textureViewer->setGeometry(QRect(440, 300, 271, 161));
        textureViewer->setFrameShape(QFrame::Box);
        textureViewer->setAlignment(Qt::AlignCenter);
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(440, 280, 67, 17));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(440, 10, 67, 17));
        MainWindow->setCentralWidget(centralWidget);
        frame_3->raise();
        frame_2->raise();
        optionsViewFrame->raise();
        imageViewer->raise();
        convertStartButton->raise();
        convertStatusProgressBar->raise();
        textureViewer->raise();
        label_8->raise();
        label_9->raise();
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "LIRS-WCT GUI", nullptr));
        imageViewer->setText(QApplication::translate("MainWindow", "Load the image", nullptr));
#ifndef QT_NO_TOOLTIP
        zAxisScaleLabel->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Set z-axis scale factor, which affects the height of output model (default = <span style=\" font-family:'monospace';\">0.25</span>)</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        zAxisScaleLabel->setText(QApplication::translate("MainWindow", "Height scale", nullptr));
#ifndef QT_NO_TOOLTIP
        dimensionScaleLabel->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Set xyz-axis scale factor, which affect the dimensions of output model (default = <span style=\" font-family:'monospace';\">0.05</span>)</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        dimensionScaleLabel->setText(QApplication::translate("MainWindow", "Sizes scale", nullptr));
#ifndef QT_NO_TOOLTIP
        xAxisRotateLabel->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Set x-axis rotate angle (default = -90 degrees)</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        xAxisRotateLabel->setText(QApplication::translate("MainWindow", "X axis rotate angle", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Scale parameters", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Sizes of generated mesh", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Width", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Length", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Height", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "Image conversion settings", nullptr));
        smoothingOptionCheckbox->setText(QApplication::translate("MainWindow", "Smoothing ", nullptr));
        sharpnessOptionCheckbox->setText(QApplication::translate("MainWindow", "Sharpness", nullptr));
#ifndef QT_NO_TOOLTIP
        colorInverseOptionCheckbox->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Using preliminary image color inverse</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        colorInverseOptionCheckbox->setText(QApplication::translate("MainWindow", "Color inverse", nullptr));
#ifndef QT_NO_TOOLTIP
        grayscaleOptionCheckbox->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Convert arbitrary image to grayscale</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        grayscaleOptionCheckbox->setText(QApplication::translate("MainWindow", "Grayscale", nullptr));
        convertStartButton->setText(QApplication::translate("MainWindow", "Convert!", nullptr));
        outputFilenameEdit->setPlaceholderText(QApplication::translate("MainWindow", "Path of output folder", nullptr));
        outputFolderPathEdit->setPlaceholderText(QApplication::translate("MainWindow", "Name of output model", nullptr));
        outputFolderSelect->setText(QApplication::translate("MainWindow", "Select output folder", nullptr));
        selectImageButton->setText(QApplication::translate("MainWindow", "Select file", nullptr));
        textureImageSelect->setText(QApplication::translate("MainWindow", "Select texture", nullptr));
        selectedTexturePath->setPlaceholderText(QApplication::translate("MainWindow", "Path of texture (optional)", nullptr));
#ifndef QT_NO_TOOLTIP
        pngToDaeSelect->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Convert png image to dae model</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        pngToDaeSelect->setText(QApplication::translate("MainWindow", "Image to Dae", nullptr));
#ifndef QT_NO_TOOLTIP
        pngToStlSelect->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Convert png file model to dae model</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        pngToStlSelect->setText(QApplication::translate("MainWindow", "Png to Stl", nullptr));
#ifndef QT_NO_TOOLTIP
        stlToDaeSelect->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Convert stl file model to dae model</p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
        stlToDaeSelect->setText(QApplication::translate("MainWindow", "Stl to Dae", nullptr));
        label->setText(QApplication::translate("MainWindow", "\320\241onversion mode", nullptr));
        textureViewer->setText(QApplication::translate("MainWindow", "Load the texture", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Texture", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Image", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
