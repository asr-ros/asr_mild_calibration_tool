/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FRAMEEDITORWINDOW_H
#define FRAMEEDITORWINDOW_H
#include <QMainWindow>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QLabel>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <Transformation/transformationfile_manager_data.h>
#include <VisualisationUtils/markerpublisher.h>

class FrameEditor : public QMainWindow
{
    Q_OBJECT
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    explicit FrameEditor(QWidget *parent = 0);

protected:
    void resizeEvent(QResizeEvent * event);

private:
    std::vector<colouredCameraFrame, Eigen::aligned_allocator<colouredCameraFrame> > colouredCameraFrames;
    std::vector<Transformation_Data> transformationData;

    QMenu *fileMenu;
    QAction *openAct;
    QAction *saveAct;
    QListWidget * lstFrames;

    QLabel *lblItemCount;

    MarkerPublisher * markerPublisher;

    int lastSelected;
signals:
public slots:

private slots:
    void openFile();
    void saveFile();
    void lstFramesItemClicked(QListWidgetItem * item);

};

#endif // FRAMEEDITORWINDOW_H
