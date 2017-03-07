/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "VisualisationUtils/laserscanwidget.h"
#include <iostream>

#include <QtGui/QApplication>
#include <QtConcurrentRun>

#include <qapplication.h>
#ifndef Q_MOC_RUN
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <ros/init.h>
#include <qwt_series_data.h>
#include <qwt_symbol.h>
#endif
#include <QLayout>

using namespace std;
    LaserScanWidget::LaserScanWidget(QWidget *parent, int appearance, unsigned int showData, LaserScanThread *currentThread) :
        QWidget(parent)
    {
        this->currentThread = currentThread;
        connect(currentThread, SIGNAL(valueChanged()),this, SLOT(replot()));
        //plot_polar = 0;
        plot_flat = 0;
        this->appearance = -1;
        setGraph(appearance, showData);
    }

    void LaserScanWidget::setGraph(int appearance, unsigned int showData)
    {
        QPen pen1(Qt::gray, 1, Qt::SolidLine);
        QPen pen2(Qt::red, 1, Qt::SolidLine);
        QPen pen3(Qt::green, 1, Qt::SolidLine);
        QPen pen4(Qt::red, 3, Qt::SolidLine);
        QPen pen5(Qt::red, 3, Qt::SolidLine);

        if (this->appearance != appearance)
        {
            /*if (plot_polar != 0)
            {
                plot_polar->setVisible(false);
                //layout()->removeWidget(plot_polar);
                delete plot_polar;
                plot_polar = 0;
            }*/
            if (plot_flat != 0)
            {
                plot_flat->setVisible(false);
                //layout()->removeWidget(plot_flat);
                delete plot_flat;
                plot_flat = 0;
            }

        this->appearance = appearance;

        if (appearance == APPEARENCE_POLAR)
        {
            /*int value_count;
            value_count = currentThread->data_field_size*(360/currentThread->angle_spread);
            plot_polar = new QwtPolarPlot(this);
            plot_polar->setAzimuthOrigin(0);

            plot_polar->setScale(QwtPolar::Azimuth, 0, value_count);
            plot_polar->setAzimuthOrigin(currentThread->starting_angle*(M_PI/180));
            plot_polar->setScale(QwtPolar::Radius, 0, 4.0);
            curve_roh_polar = new QwtPolarCurve();
            curve_roh_polar->setPen(pen1);
            curve_roh_polar->setSymbol(new QwtSymbol(QwtSymbol::Ellipse,QBrush(Qt::blue),QPen (Qt::blue),QSize(1,1)));
            seriesData_raw = new PolarPointSeriesData(&(currentThread->dx[0]), &(currentThread->data_raw[0]), currentThread->data_field_size);
            curve_roh_polar->setData(seriesData_raw);

            curve_gemittelt_polar = new QwtPolarCurve();
            curve_gemittelt_polar->setPen(pen2);
            curve_gemittelt_polar->setSymbol(new QwtSymbol(QwtSymbol::Ellipse,QBrush(Qt::blue),QPen (Qt::blue),QSize(1,1)));
            seriesData_avg = new PolarPointSeriesData(&(currentThread->dx[0]), &(currentThread->data_avg[0]), currentThread->data_field_size);
            curve_gemittelt_polar->setData(seriesData_avg);

            curve_gauss_polar = new QwtPolarCurve();
            curve_gauss_polar->setPen(pen3);
            curve_gauss_polar->setSymbol(new QwtSymbol(QwtSymbol::Ellipse,QBrush(Qt::blue),QPen (Qt::blue),QSize(1,1)));
            seriesData_filtered = new PolarPointSeriesData(&(currentThread->dx[0]), &(currentThread->data_filtered[0]), currentThread->data_field_size);
            curve_gauss_polar->setData(seriesData_filtered);

            curve_erkannt_polar = new QwtPolarCurve();
            curve_erkannt_polar->setPen(pen4);
            curve_erkannt_polar->setSymbol(new QwtSymbol(QwtSymbol::Ellipse,QBrush(Qt::blue),QPen (Qt::blue),QSize(1,1)));
            seriesData_segments = new PolarPointSeriesData(&(currentThread->dx[0]), &(currentThread->data_segments[0]), currentThread->data_field_size);
            curve_erkannt_polar->setData(seriesData_segments);

            curve_ecke_polar = new QwtPolarCurve();
            curve_ecke_polar->setPen(pen5);
            curve_ecke_polar->setSymbol(new QwtSymbol(QwtSymbol::Ellipse,QBrush(Qt::blue),QPen (Qt::blue),QSize(1,1)));
            seriesData_edges = new PolarPointSeriesData(&(currentThread->dx[0]), &(currentThread->data_edges[0]), currentThread->data_field_size);
            curve_ecke_polar->setData(seriesData_edges);

            plot_polar->setAutoReplot(true);
            plot_polar->show();
            plot_polar->resize(this->width(),this->height());*/
        }
        else if (appearance == APPEARENCE_FLAT)
        {
            plot_flat = new QwtPlot(this);
            plot_flat->setAxisScale(QwtPlot::yLeft, 0,6);
            plot_flat->setAxisScale(QwtPlot::xBottom, currentThread->starting_angle,currentThread->starting_angle + currentThread->angle_spread,30.0);
            //plot_flat->setAxisScale(QwtPlot::xBottom, currentThread->starting_angle,currentThread->starting_angle + currentThread->data_field_size * currentThread->angle_spread);
                curve_roh_flat = new QwtPlotCurve();
                curve_roh_flat->setPen(pen1);
                curve_roh_flat->setRawSamples(currentThread->dx, currentThread->data_raw, currentThread->data_field_size);

                curve_gemittelt_flat = new QwtPlotCurve();
                curve_gemittelt_flat->setPen(pen2);
                curve_gemittelt_flat->setRawSamples(currentThread->dx, currentThread->data_avg, currentThread->data_field_size);

                curve_gauss_flat = new QwtPlotCurve();
                curve_gauss_flat->setPen(pen3);
                curve_gauss_flat->setRawSamples(currentThread->dx, currentThread->data_filtered, currentThread->data_field_size);

                curve_erkannt_flat = new QwtPlotCurve();
                curve_erkannt_flat->setPen(pen4);
                curve_erkannt_flat->setRawSamples(currentThread->dx, currentThread->data_segments, currentThread->data_field_size);

                curve_ecke_flat = new QwtPlotCurve();
                curve_ecke_flat->setPen(pen5);
                curve_ecke_flat->setRawSamples(currentThread->dx, currentThread->data_edges, currentThread->data_field_size);

            plot_flat->setAutoReplot(true);
            plot_flat->show();
            plot_flat->resize(this->width(),this->height());
        }
        setCurves(showData);
        }
    }

    void LaserScanWidget::replot()
    {
        if (this->appearance == APPEARENCE_POLAR)
        {
            /*seriesData_raw->update(&(currentThread->dx[0]), &(currentThread->data_raw[0]), currentThread->data_field_size);
            seriesData_avg->update(&(currentThread->dx[0]), &(currentThread->data_avg[0]), currentThread->data_field_size);
            seriesData_filtered->update(&(currentThread->dx[0]), &(currentThread->data_filtered[0]),currentThread->data_field_size);
            seriesData_segments->update(&(currentThread->dx[0]), &(currentThread->data_segments[0]), currentThread->data_field_size);
            seriesData_edges->update(&(currentThread->dx[0]), &(currentThread->data_edges[0]), currentThread->data_field_size);
            plot_polar->replot();*/
        }
        else if (appearance == APPEARENCE_FLAT)
        {
            plot_flat->replot();
        }
    }


   void LaserScanWidget::setCurves(unsigned int showData)
   {
       if (this->appearance == APPEARENCE_POLAR)
       {
           /*if ((showData & 1) != 0)
           {
               curve_roh_polar->attach(plot_polar);
               curve_roh_polar->show();
           }
           else
           {
               curve_roh_polar->detach();
           }
           if ((showData & 2) != 0)
           {
               curve_gemittelt_polar->attach(plot_polar);
               curve_gemittelt_polar->show();
           }
           else
           {
               curve_gemittelt_polar->detach();
           }
           if ((showData & 4) != 0)
           {
               curve_gauss_polar->attach(plot_polar);
               curve_gauss_polar->show();
           }
           else
           {
               curve_gauss_polar->detach();
           }
           if ((showData & 8) != 0)
           {
               curve_erkannt_polar->attach(plot_polar);
               curve_erkannt_polar->show();
           }
           else
           {
               curve_erkannt_polar->detach();
           }
           if ((showData & 16) != 0)
           {
               curve_ecke_polar->attach(plot_polar);
               curve_ecke_polar->show();
           }
           else
           {
               curve_ecke_polar->detach();
           }
           plot_polar->replot();*/
       }
       else if (appearance == APPEARENCE_FLAT)
       {
           if ((showData & 1) != 0)
           {
               curve_roh_flat->attach(plot_flat);
               curve_roh_flat->show();
           }
           else
           {
               curve_roh_flat->detach();
           }
           if ((showData & 2) != 0)
           {
               curve_gemittelt_flat->attach(plot_flat);
               curve_gemittelt_flat->show();
           }
           else
           {
               curve_gemittelt_flat->detach();
           }
           if ((showData & 4) != 0)
           {
               curve_gauss_flat->attach(plot_flat);
               curve_gauss_flat->show();
           }
           else
           {
               curve_gauss_flat->detach();
           }
           if ((showData & 8) != 0)
           {
               curve_erkannt_flat->attach(plot_flat);
               curve_erkannt_flat->show();
           }
           else
           {
               curve_erkannt_flat->detach();
           }
           if ((showData & 16) != 0)
           {
               curve_ecke_flat->attach(plot_flat);
               curve_ecke_flat->show();
           }
           else
           {
               curve_ecke_flat->detach();
           }
           plot_flat->replot();
       }
   }

   void LaserScanWidget::resizeEvent(QResizeEvent * event)
   {
       if (this->appearance == APPEARENCE_POLAR)
       {
           //plot_polar->resize(this->width(),this->height());
       }
       else
       {
           plot_flat->resize(this->width(),this->height());
       }
       QWidget::resizeEvent(event);
   }



