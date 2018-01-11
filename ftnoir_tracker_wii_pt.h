/* Copyright (c) 2012 Patrick Ruoff
 * Copyright (c) 2014-2016 Stanislaw Halik <sthalik@misaki.pl>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */
 /*
 * copyright (c) 2017-2018 Wei Shuai <cpuwolf@gmail.com>
 * WIImote support
 */
#pragma once

#include "api/plugin-api.hpp"
#include "ftnoir_tracker_wii_pt_settings.h"

#include <atomic>

#include "cv/numeric.hpp"

#include "camera.h"
#include "point_extractor.h"
#include "point_tracker.h"
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
#include <wiiyourself/wiimote.h>

class TrackerDialog_WII_PT;

namespace impl {

using namespace types;

class Tracker_WII_PT : public QThread, public ITracker
{
    Q_OBJECT
    friend class camera_dialog;
    friend class ::TrackerDialog_WII_PT;
public:
    Tracker_WII_PT();
    ~Tracker_WII_PT() override;
    void start_tracker(QFrame* parent_window) override;
    void data(double* data) override;

    Affine pose();
    int  get_n_points();
    bool get_cam_info(CamInfo* info);
public slots:
    void maybe_reopen_camera();
    void set_fov(int value);
protected:
    void run() override;
private:
	wiimote * m_pDev;
	static void on_state_change(wiimote &remote,
		state_change_flags changed,
		const wiimote_state &new_state);
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
    //PointExtractor point_extractor;
    PointTracker   point_tracker;

    qshared<cv_video_widget> video_widget;
    qshared<QLayout> layout;

    settings_pt s;
    cv::Mat frame, preview_frame;
    std::vector<vec2> points;

    QSize preview_size;

    std::atomic<unsigned> point_count;
    std::atomic<unsigned char> commands;
    std::atomic<bool> ever_success;

    static constexpr f rad2deg = f(180/M_PI);
    //static constexpr float deg2rad = float(M_PI/180);
};

} // ns impl

class PT_metadata : public Metadata
{
    QString name() { return QString(QCoreApplication::translate("PT_metadata", "WiiPointTracker 1.0")); }
    QIcon icon() { return QIcon(":/Resources/wii.png"); }
};

using impl::Tracker_WII_PT;
