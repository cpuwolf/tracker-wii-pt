/* Copyright (c) 2012 Patrick Ruoff
 * Copyright (c) 2014-2016 Stanislaw Halik <sthalik@misaki.pl>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#pragma once

#include "api/plugin-api.hpp"
#include "ftnoir_tracker_wii_pt_settings.h"

#include "numeric.hpp"

#include "camera.h"
#include "point_extractor.h"
#include "point_tracker.h"
#include "compat/timer.hpp"
#include "cv/video-widget.hpp"
#include "compat/util.hpp"

#include <QCoreApplication>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QTime>
#include <QLayout>
#include <QSize>
#include <atomic>
#include <memory>
#include <vector>

class TrackerDialog_PT;

namespace impl {

using namespace types;

class Tracker_PT : public QThread, public ITracker
{
    Q_OBJECT
    friend class camera_dialog;
    friend class ::TrackerDialog_PT;
public:
    Tracker_PT();
    ~Tracker_PT() override;
    void start_tracker(QFrame* parent_window) override;
    void data(double* data) override;

    Affine pose();
    int  get_n_points();
    bool get_cam_info(CamInfo* info);
public slots:
    void apply_settings();
protected:
    void run() override;
private:
    // thread commands
    enum Command : unsigned char
    {
        ABORT = 1<<0
    };
    void set_command(Command command);
    void reset_command(Command command);

    QMutex camera_mtx;
    QMutex data_mtx;
    Camera       camera;
    PointExtractor point_extractor;
    PointTracker   point_tracker;

    qshared<cv_video_widget> video_widget;
    qshared<QLayout> layout;

    settings_pt s;
    Timer time;
    cv::Mat frame, preview_frame;
    std::vector<vec2> points;

    QSize preview_size;

    volatile unsigned point_count;
    volatile unsigned char commands;
    volatile bool ever_success;

    static constexpr f rad2deg = f(180/M_PI);
    //static constexpr float deg2rad = float(M_PI/180);
};

} // ns impl

class PT_metadata : public Metadata
{
    QString name() { return QString(QCoreApplication::translate("PT_metadata", "PointTracker 1.1")); }
    QIcon icon() { return QIcon(":/Resources/Logo_IR.png"); }
};

using impl::Tracker_PT;
