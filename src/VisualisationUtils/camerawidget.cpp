/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "VisualisationUtils/camerawidget.h"

CameraWidget::CameraWidget(QWidget *parent, CameraThread *camera):
    QWidget(parent)
{
    this->camera = camera;
    painterInit = false;
    imageInit =false;

    image = new QImage(1, 1, QImage::Format_RGB16);

    connect(camera, SIGNAL(imageReceived(const sensor_msgs::Image::ConstPtr&)),this, SLOT(imageReceived(const sensor_msgs::Image::ConstPtr&)));
    camera->start();
}

void CameraWidget::imageReceived(const sensor_msgs::Image::ConstPtr& msg)
{
    if (!imageInit) {image = new QImage(msg->width, msg->height, QImage::Format_RGB16);}
    imageInit = true;

    for(int i = 0; i < image->width(); i++)
    {
        for(int j = 0; j < image->height(); j++)
        {
            image->setPixel(i, j, qRgb(msg->data[j * msg->width + i+2],msg->data[j * msg->width + i+1],msg->data[j * msg->width + i]));
        }
    }

    this->repaint();
}

void CameraWidget::paintEvent(QPaintEvent * pe)
{
    QWidget::paintEvent(pe);
    if (!painterInit) {painter = new QPainter(this);}
    painterInit = true;

    if (imageInit)
    {
        QPainter painter(this);
        painter.drawImage(QRect(0,0, this->width(), this->height()), *image);
    }
}
void CameraWidget::resizeEvent(QResizeEvent * event)
{
    QWidget::resizeEvent(event);
}
