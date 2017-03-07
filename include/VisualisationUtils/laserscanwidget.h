/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef LASERSCANWIDGET_H
#define LASERSCANWIDGET_H
#ifndef Q_MOC_RUN
#include <qwt_plot_curve.h>
#include <qwt_plot.h>
#endif
#include <QWidget>
#include "laserscanwidget.h"
#include "laserscanthread.h"
#include "polarpointseriesdata.h"
//#include <qwt_polar_plot.h>
//#include <qwt_polar_curve.h>

class LaserScanWidget : public QWidget
{
        Q_OBJECT
    protected:
        QwtPlotCurve *curve_roh_flat;
        //QwtPolarCurve *curve_roh_polar;
        QwtPlotCurve *curve_gemittelt_flat;
        //QwtPolarCurve *curve_gemittelt_polar;
        QwtPlotCurve *curve_gauss_flat;
        //QwtPolarCurve *curve_gauss_polar;
        QwtPlotCurve *curve_erkannt_flat;
        //QwtPolarCurve *curve_erkannt_polar;
        QwtPlotCurve *curve_ecke_flat;
        //QwtPolarCurve *curve_ecke_polar;
        QwtPlot *plot_flat;
        //QwtPolarPlot *plot_polar;
        LaserScanThread *currentThread;

        /*PolarPointSeriesData *seriesData_raw;
        PolarPointSeriesData *seriesData_avg;
        PolarPointSeriesData *seriesData_filtered;
        PolarPointSeriesData *seriesData_segments;
        PolarPointSeriesData *seriesData_edges;*/
        int appearance;

        void resizeEvent(QResizeEvent * event);
    public:
        explicit LaserScanWidget(QWidget *parent = 0, int appearance = APPEARENCE_FLAT, unsigned int showData = 1, LaserScanThread *currentThread = 0);
        void setGraph(int appearance, unsigned int showData);
        static const int APPEARENCE_FLAT = 0;
        static const int APPEARENCE_POLAR = 1;

    signals:

    public slots:
        void replot();
        void setCurves(unsigned int showData);
};

#endif // LASERSCANWIDGET_H
