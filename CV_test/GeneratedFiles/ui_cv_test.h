/********************************************************************************
** Form generated from reading UI file 'cv_test.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CV_TEST_H
#define UI_CV_TEST_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CV_testClass
{
public:
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QListWidget *listWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *loadButton;
    QPushButton *parseButton;
    QPushButton *processButton;
    QPushButton *pushButton;
    QPushButton *solverButton;
    QLabel *label;
    QProgressBar *progressBar;

    void setupUi(QWidget *CV_testClass)
    {
        if (CV_testClass->objectName().isEmpty())
            CV_testClass->setObjectName(QStringLiteral("CV_testClass"));
        CV_testClass->resize(900, 808);
        verticalLayout_2 = new QVBoxLayout(CV_testClass);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        listWidget = new QListWidget(CV_testClass);
        listWidget->setObjectName(QStringLiteral("listWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(listWidget->sizePolicy().hasHeightForWidth());
        listWidget->setSizePolicy(sizePolicy);
        listWidget->setSizeIncrement(QSize(0, 0));

        horizontalLayout_3->addWidget(listWidget);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        loadButton = new QPushButton(CV_testClass);
        loadButton->setObjectName(QStringLiteral("loadButton"));

        horizontalLayout_2->addWidget(loadButton);

        parseButton = new QPushButton(CV_testClass);
        parseButton->setObjectName(QStringLiteral("parseButton"));

        horizontalLayout_2->addWidget(parseButton);

        processButton = new QPushButton(CV_testClass);
        processButton->setObjectName(QStringLiteral("processButton"));

        horizontalLayout_2->addWidget(processButton);

        pushButton = new QPushButton(CV_testClass);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        horizontalLayout_2->addWidget(pushButton);

        solverButton = new QPushButton(CV_testClass);
        solverButton->setObjectName(QStringLiteral("solverButton"));

        horizontalLayout_2->addWidget(solverButton);


        verticalLayout->addLayout(horizontalLayout_2);

        label = new QLabel(CV_testClass);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
        label->setMaximumSize(QSize(1600, 1000));

        verticalLayout->addWidget(label);

        verticalLayout->setStretch(0, 1);
        verticalLayout->setStretch(1, 10);

        horizontalLayout_3->addLayout(verticalLayout);

        horizontalLayout_3->setStretch(0, 1);
        horizontalLayout_3->setStretch(1, 4);

        verticalLayout_2->addLayout(horizontalLayout_3);

        progressBar = new QProgressBar(CV_testClass);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setValue(0);

        verticalLayout_2->addWidget(progressBar);


        retranslateUi(CV_testClass);

        QMetaObject::connectSlotsByName(CV_testClass);
    } // setupUi

    void retranslateUi(QWidget *CV_testClass)
    {
        CV_testClass->setWindowTitle(QApplication::translate("CV_testClass", "CV_test", 0));
        loadButton->setText(QApplication::translate("CV_testClass", "Load picture", 0));
        parseButton->setText(QApplication::translate("CV_testClass", "Parsing step", 0));
        processButton->setText(QApplication::translate("CV_testClass", "Postprocessing step", 0));
        pushButton->setText(QApplication::translate("CV_testClass", "Get matching coefficients", 0));
        solverButton->setText(QApplication::translate("CV_testClass", "Solve", 0));
        label->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class CV_testClass: public Ui_CV_testClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CV_TEST_H
