#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private slots:
    void on_moving_pointcloud_toolButton_clicked();
    void on_fixed_pointcloud_toolButton_clicked();
    void on_register_pushButton_clicked();

private:
    Ui::Widget *ui;
    QString fixed_pointcloud_filename_;
    QString moving_pointcloud_filename_;


};

#endif // WIDGET_H
