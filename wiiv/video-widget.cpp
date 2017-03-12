/* Copyright (c) 2012 Patrick Ruoff
 * Copyright (c) 2014-2016 Stanislaw Halik <sthalik@misaki.pl>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#include "video-widget.hpp"
#include <opencv2/imgproc.hpp>

wiiv_video_widget::wiiv_video_widget(QWidget* parent) :
    QWidget(parent), freshp(false)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(update_and_repaint()));
    timer.start(65);
}

void wiiv_video_widget::update_image(const cv::Mat& frame)
{
    QMutexLocker l(&mtx);

    if (!freshp)
    {
        const int w = preview_size.width(), h = preview_size.height();

        if (w < 1 || h < 1)
            return;

        if (_frame.cols != frame.cols || _frame.rows != frame.rows)
            _frame = cv::Mat(frame.rows, frame.cols, CV_8UC3);
        frame.copyTo(_frame);
        freshp = true;

        if (_frame2.cols != _frame.cols || _frame2.rows != _frame.rows)
            _frame2 = cv::Mat(_frame.rows, _frame.cols, CV_8UC3);

        if (_frame3.cols != w || _frame3.rows != h)
            _frame3 = cv::Mat(h, w, CV_8UC3);

        cv::cvtColor(_frame, _frame2, cv::COLOR_RGB2BGR);

        const cv::Mat* img_;

        if (_frame.cols != w || _frame.rows != h)
        {
            cv::resize(_frame2, _frame3, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);

            img_ = &_frame3;
        }
        else
            img_ = &_frame2;

        const cv::Mat& img = *img_;

        texture = QImage((const unsigned char*) img.data, w, h, QImage::Format_RGB888);
    }
}

void wiiv_video_widget::paintEvent(QPaintEvent*)
{
    QMutexLocker foo(&mtx);
    QPainter painter(this);
    painter.drawImage(rect(), texture);
}

void wiiv_video_widget::update_and_repaint()
{
    QMutexLocker l(&mtx);

    preview_size = size();

    if (freshp)
    {
        freshp = false;
        update();
    }
}
