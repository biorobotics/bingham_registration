#include "widget.h"
#include "ui_widget.h"
#include <QFileDialog>

#include <iostream>
#include <fstream>
#include <cstring>
#include <ctime>
#include "registration_est_bingham_kf_rgbd.h"

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_moving_pointcloud_toolButton_clicked()
{
    moving_pointcloud_filename_ = QFileDialog::getOpenFileName(this,
         tr("Open moving pointcloud file"), "/home", tr("txt Files (*.txt)"));
    ui->moving_pointcloud_filename_lineEdit->setText(moving_pointcloud_filename_);

}

void Widget::on_fixed_pointcloud_toolButton_clicked()
{
    fixed_pointcloud_filename_ = QFileDialog::getOpenFileName(this,
         tr("Open fixed pointcloud file"), "/home", tr("txt Files (*.txt)"));
    ui->fixed_pointcloud_filename_lineEdit->setText(fixed_pointcloud_filename_);

}

void Widget::on_register_pushButton_clicked()
{
    string movingFileString = fixed_pointcloud_filename_.toUtf8().constData();
    string fixedFileString = moving_pointcloud_filename_.toUtf8().constData();

    // Read .text files for data points
    ifstream sensedFile;
    ifstream CADFile;

    cout << "\nStatic point cloud: " << fixedFileString << "\nMoving point cloud: " << movingFileString << endl;

    sensedFile.open(movingFileString, ifstream::in);
    CADFile.open(fixedFileString, ifstream::in);
    
    if (!sensedFile.good() || !CADFile.good()) {
        cout << "Files " << fixedFileString << ", " << movingFileString << " not found" << "\n";
    } 
    
    // Vector for appending points
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> pointVector;
    
    // read sensedFile into ptcldMoving
    while (!sensedFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        sensedFile.getline(buf, MAX_CHARS_PER_LINE);
        // store line in a vector
        istringstream iss(buf);
        Vector3d temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        // Make sure all three were read in correctly
        if(iss.fail())
            call_error(movingFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingFileString + ": Input data doesn't match dimension (too many per line)");
        
        // Add temp to list of vectors  
        pointVector.push_back(temp);
    }

    PointCloud ptcldMoving(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        ptcldMoving.col(i) = pointVector[i];
    
    pointVector.clear();

    // read CADFile into ptcldFixed
    while (!CADFile.eof()) {
                // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        CADFile.getline(buf, MAX_CHARS_PER_LINE);
        // store line in a vector
        std::istringstream iss(buf);
        Vector3d temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        if(iss.fail()) 
            call_error(fixedFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingFileString + ": Input data doesn't match dimension (too many per line)");
        // Add temp to list of vectors
        pointVector.push_back(temp);
    }

    PointCloud ptcldFixed(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        ptcldFixed.col(i) = pointVector[i];
    
    pointVector.clear();
    pointVector.shrink_to_fit();
    
    sensedFile.close();
    CADFile.close();
    
    double timeSum = 0;
    ofstream myFile;
    myFile.open("Result_temp.txt");
    for (int i = 0; i < 10; i++) {
        clock_t begin = clock();    // For timing the performance

        // Run the registration function

        struct RegistrationResult result = registration_est_bingham_kf_rgbd(ptcldMoving, 
                                                                            ptcldFixed);

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        timeSum += elapsed_secs;
    
        cout << "Xreg:" << endl << result.Xreg.transpose() << endl << endl;
        cout << "Xregsave:" << endl << result.Xregsave.transpose() << endl;
        if (i == 9)
        {
            myFile << "Xreg:" << endl << result.Xreg.transpose() << endl << endl;
            myFile << "Xregsave:" << endl << result.Xregsave.transpose() << endl << endl;
        }
    }
    cout << "Average registration runtime is: " << timeSum / 10 << " seconds." << endl << endl;
    myFile << "Average registration runtime is: " << timeSum / 10 << " seconds." << endl;
    myFile.close();
}
