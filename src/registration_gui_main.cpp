#include <iostream>
#include <fstream>
#include <cstring>
#include <ctime>
#include "registration_est_bingham_kf_rgbd.h"

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";

// main:
// Read sensed data and CAD data from .txt files and use them as input for
// running the registration function


#include "widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();

    return a.exec();
}

