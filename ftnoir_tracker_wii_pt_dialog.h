/* Copyright (c) 2012 Patrick Ruoff
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#ifndef FTNOIR_Tracker_WII_PT_DIALOG_H
#define FTNOIR_Tracker_WII_PT_DIALOG_H

#include "api/plugin-api.hpp"
#include "ftnoir_tracker_wii_pt_settings.h"
#include "ftnoir_tracker_wii_pt.h"
#include "ui_FTNoIR_WII_PT_Controls.h"
#include "cv/translation-calibrator.hpp"
#include "cv/video-widget.hpp"

#include <QTimer>
#include <QMutex>

class TrackerDialog_WII_PT : public ITrackerDialog
{
    Q_OBJECT
public:
    TrackerDialog_WII_PT();
    void register_tracker(ITracker *tracker) override;
    void unregister_tracker() override;
    void save();
public slots:
    void doOK();
    void doCancel();

    void startstop_trans_calib(bool start);
    void trans_calib_step();
    void poll_tracker_info_impl();
    void set_camera_settings_available(const QString& camera_name);
    void show_camera_settings();
signals:
    void poll_tracker_info();
private:
    settings_pt s;
    Tracker_WII_PT* tracker;
    QTimer timer, calib_timer;
    TranslationCalibrator trans_calib;
    QMutex calibrator_mutex;

    Ui::UICWIIPTClientControls ui;
};

#endif //FTNOIR_Tracker_WII_PT_DIALOG_H
