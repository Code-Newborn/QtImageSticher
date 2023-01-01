#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDragEnterEvent>
#include <QMimeData>

#include "sticher/surfMethod.h"
#include "sticher/siftMethod.h"
#include "sticher/stichMethod.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

typedef struct
{
    cv::Point2f left_top;
    cv::Point2f left_bottom;
    cv::Point2f right_top;
    cv::Point2f right_bottom;
} four_corners_t;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    virtual bool eventFilter(QObject *watched, QEvent *event) override;

private:
    Ui::MainWindow *ui;

    cv::Mat imgMat1;
    cv::Mat imgMat2;
};
#endif // MAINWINDOW_H


