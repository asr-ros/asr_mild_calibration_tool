/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <GUI/frameeditorwindow.h>
#include <QFileDialog>

FrameEditor::FrameEditor(QWidget *parent)
{
    markerPublisher = new MarkerPublisher(0);

    //Set Toolbar
    fileMenu = this->menuBar()->addMenu("&File");
    openAct = new QAction(tr("&Open"), this);
    openAct->setShortcuts(QKeySequence::Open);
    openAct->setStatusTip(tr("Open a file"));
    connect(openAct, SIGNAL(triggered()), this, SLOT(openFile()));
    fileMenu->addAction(openAct);
    saveAct = new QAction(tr("&Save"), this);
    saveAct->setShortcuts(QKeySequence::Save);
    saveAct->setStatusTip(tr("Save file"));
    connect(saveAct, SIGNAL(triggered()), this, SLOT(saveFile()));
    fileMenu->addAction(saveAct);

    lblItemCount = new QLabel(this);
    lblItemCount->resize(200,30);

    //Set Listbox
    lstFrames = new QListWidget(this);
    lstFrames->move(20,30);
    lstFrames->setVisible(true);
    lstFrames->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(lstFrames, SIGNAL(itemClicked(QListWidgetItem*)),  this, SLOT(lstFramesItemClicked(QListWidgetItem*)));
}

void FrameEditor::resizeEvent(QResizeEvent * event)
{
    lstFrames->resize(this->width()-40, this->height()-160);
    lblItemCount->move(20, this->height()-150);
    QMainWindow::resizeEvent(event);
}

void FrameEditor::lstFramesItemClicked(QListWidgetItem * item)
{
    int row = item->listWidget()->row( item );
    colouredCameraFrames.at(row).color.r = 1.0;
    colouredCameraFrames.at(row).color.g = 0.0;
    if (lastSelected > -1 )
    {
        colouredCameraFrames.at(lastSelected).color.r = 0.0;
        colouredCameraFrames.at(lastSelected).color.g = 1.0;
    }
    lastSelected = row;
    markerPublisher->publishColouredCameraFrames(&colouredCameraFrames);
}


void FrameEditor::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open File", "CalibrationData.data", tr("Calibrationfile (*.data)"));
    if (fileName.length() > 0)
    {
        ROS_INFO_STREAM("File: " << fileName.toStdString());
        TransformationFile_Manager_Data fileManager(fileName.toStdString());
        transformationData = fileManager.readFromFile();
        lstFrames->clear();
        colouredCameraFrames.clear();
        for (unsigned int i = 0; i < transformationData.size(); i++)
        {
            Transformation_Data data = transformationData.at(i);
            QString name = QString::number(data.pan) + ", " + QString::number(data.tilt);
            new QListWidgetItem(name, lstFrames);
            colouredCameraFrame cCF;
            cCF.pose = data.PTU_Frame;
            ROS_INFO_STREAM(data.PTU_Frame(0,3) << ", " << data.PTU_Frame(1,3) << ", " << data.PTU_Frame(2,3));
            cCF.color.r = 0.0;
            cCF.color.g = 1.0;
            cCF.color.b = 0.0;
            cCF.color.a = 1.0;
            colouredCameraFrames.push_back(cCF);
        }
        lastSelected = -1;
        markerPublisher->publishColouredCameraFrames(&colouredCameraFrames);
    }
}

void FrameEditor::saveFile()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Save file", "CalibrationData.data", tr("Calibrationfile (*.data)"));
    if (fileName.length() > 0)
    {
        ROS_INFO_STREAM("File: " << fileName.toStdString());
        TransformationFile_Manager_Data fileManager(fileName.toStdString());
        fileManager.writeToFile(transformationData);
    }
}
