#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->label_img1->installEventFilter(this);
    ui->label_img2->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->label_img1) // 左图
    {

        if (event->type() == QEvent::DragEnter) // 拖放时鼠标进入
        {
            QDragEnterEvent *dee = dynamic_cast<QDragEnterEvent *>(event);
            dee->acceptProposedAction(); // 用户可以在这个窗口部件上拖放对象
            return true;
        }
        else if (event->type() == QEvent::Drop) // 拖放放下鼠标
        {
            QDropEvent *de = dynamic_cast<QDropEvent *>(event);
            QList<QUrl> urls = de->mimeData()->urls(); // 获取拖放文件的路径
            if (urls.isEmpty())
            {
                return true;
            }
            QString path = urls.first().toLocalFile(); // 获取图片路径
            QImage image(path);                        // 加载图片

            if (!image.isNull())
            {
                imgMat1 = cv::imread(path.toStdString());
                image = image.scaled(ui->label_img1->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation); // 处理图片大小
                ui->label_img1->setPixmap(QPixmap::fromImage(image));                                        // 显示图片
            }

            if ((ui->label_img1->pixmap() != 0) && (ui->label_img2->pixmap() != 0)) // 两图片框均存在图片
            {
                cv::Mat result;
                SurfStich(imgMat1, imgMat2, result);

                cv::cvtColor(result, result, cv::COLOR_BGR2RGB);
                QImage qimageDst(result.data, result.cols, result.rows, result.step, QImage::Format_RGB888);

                qimageDst = qimageDst.scaled(ui->label_stichImg->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                ui->label_stichImg->setPixmap(QPixmap::fromImage(qimageDst));
            }
            return true;
        }
    }

    if (watched == ui->label_img2) // 右图
    {
        if (event->type() == QEvent::DragEnter) // 拖放时鼠标进入
        {
            QDragEnterEvent *dee = dynamic_cast<QDragEnterEvent *>(event);
            dee->acceptProposedAction();
            return true;
        }
        else if (event->type() == QEvent::Drop) // 拖放放下鼠标
        {
            QDropEvent *de = dynamic_cast<QDropEvent *>(event);
            QList<QUrl> urls = de->mimeData()->urls();
            if (urls.isEmpty()) // 拖放的文件路径为空
            {
                return true;
            }

            QString path = urls.first().toLocalFile(); // 访问第一个文件路径
            QImage image(path);                        // 加载路径图片
            if (!image.isNull())
            {
                imgMat2 = cv::imread(path.toStdString());
                image = image.scaled(ui->label_img2->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                ui->label_img2->setPixmap(QPixmap::fromImage(image));
            }

            if ((ui->label_img1->pixmap() != 0) && (ui->label_img2->pixmap() != 0)) // 两图片框均存在图片
            {
                cv::Mat result;
                SurfStich(imgMat1, imgMat2, result);

                cv::cvtColor(result, result, cv::COLOR_BGR2RGB);
                QImage qimageDst(result.data, result.cols, result.rows, result.step, QImage::Format_RGB888);

                qimageDst = qimageDst.scaled(ui->label_stichImg->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                ui->label_stichImg->setPixmap(QPixmap::fromImage(qimageDst));
            }
            return true;
        }
    }
    return QWidget::eventFilter(watched, event); // 非目标控件
}
