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

#include "ftnoir_tracker_wii_pt.h"
#include "compat/camera-names.hpp"
#include "compat/math-imports.hpp"
#include <QHBoxLayout>
#include <cmath>
#include <QDebug>
#include <QFile>
#include <QCoreApplication>
#include <functional>

Tracker_WII_PT::Tracker_WII_PT() :
      point_count(0),
      commands(0),
      ever_success(false)
{
    cv::setBreakOnError(true);

    connect(s.b.get(), SIGNAL(saving()), this, SLOT(maybe_reopen_camera()), Qt::DirectConnection);
    connect(&s.fov, SIGNAL(valueChanged(int)), this, SLOT(set_fov(int)), Qt::DirectConnection);
    set_fov(s.fov);
}

Tracker_WII_PT::~Tracker_WII_PT()
{
    set_command(ABORT);
    wait();

    QMutexLocker l(&camera_mtx);
    //camera.stop();
}

void Tracker_WII_PT::set_command(Command command)
{
    //QMutexLocker lock(&mutex);
    commands |= command;
}

void Tracker_WII_PT::reset_command(Command command)
{
    //QMutexLocker lock(&mutex);
    commands &= ~command;
}

void Tracker_WII_PT::on_state_change(wiimote &remote,
	state_change_flags  changed,
	const wiimote_state &new_state)
{
	// the wiimote just connected
	if (changed & CONNECTED)
	{
		if (new_state.ExtensionType != wiimote::BALANCE_BOARD)
		{
			if (new_state.bExtension)
				remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT); // no IR dots
			else
				remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);		//    IR dots
		}
	}
	// another extension was just connected:
	else if (changed & EXTENSION_CONNECTED)
	{

		Beep(1000, 200);

		// switch to a report mode that includes the extension data (we will
		//  loose the IR dot sizes)
		// note: there is no need to set report types for a Balance Board.
		if (!remote.IsBalanceBoard())
			remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT);
	}
	else if (changed & EXTENSION_DISCONNECTED)
	{

		Beep(200, 300);

		// use a non-extension report mode (this gives us back the IR dot sizes)
		remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);
	}
}

void Tracker_WII_PT::run() {

	cv::setNumThreads(0);

	BOOL blExitThread = FALSE;
	unsigned char *pDataSource;
	MSG msg;

	m_pDev = new wiimote;
	if (m_pDev == NULL)
	{
		return;
	}
	m_pDev->ChangedCallback = on_state_change;
	m_pDev->CallbackTriggerFlags = (state_change_flags)(CONNECTED |
		EXTENSION_CHANGED |
		MOTIONPLUS_CHANGED);

	//create a blank frame
	cv::Mat blank_frame(preview_size.width(), preview_size.height(), CV_8UC3, cv::Scalar(0, 0, 0));
	char txtbuf[64];
	sprintf(txtbuf, "%s", "wait for WIImote");

reconnect:
	qDebug() << "wii wait";
	while (!m_pDev->Connect(wiimote::FIRST_AVAILABLE)) {
		if (commands & ABORT)
			goto goodbye;
		Beep(500, 20); Sleep(1500);
		cv::resize(blank_frame, preview_frame, cv::Size(preview_size.width(), preview_size.height()), 0, 0, cv::INTER_NEAREST);
		//draw wait text
		cv::putText(preview_frame,
			txtbuf,
			cv::Point(preview_frame.cols / 10, preview_frame.rows / 2),
			cv::FONT_HERSHEY_SIMPLEX,
			1,
			cv::Scalar(255, 255, 255),
			1);
		video_widget->update_image(preview_frame);
	}

	/* wiimote connected */
	m_pDev->SetLEDs(0x0f);
	Beep(1000, 300); Sleep(500);

	//m_pDev->SetRumble(true);
	

	qDebug() << "wii connected";

	while ((commands & ABORT) == 0)
	{
		while (m_pDev->RefreshState() == NO_CHANGE) {
				Sleep(1); // don't hog the CPU if nothing changed
		}
		if (commands & ABORT)
			goto goodbye;

		// did we loose the connection?
		if (m_pDev->ConnectionLost())
		{
			goto reconnect;
		}
	
		CamInfo cam_info;

		get_cam_info(&cam_info);

		//create preview video frame
		cv::resize(blank_frame, preview_frame, cv::Size(preview_size.width(), preview_size.height()), 0, 0, cv::INTER_NEAREST);

		//draw battery status
		cv::line(preview_frame,
				cv::Point(0, 0),
				cv::Point(preview_frame.cols*m_pDev->BatteryPercent/100, 0),
				(m_pDev->bBatteryDrained?cv::Scalar(255,0, 0): cv::Scalar(0, 80, 0)),
				2);
		{
			//draw horizon
			static int pdelta = 0;
			static int rdelta = 0;
			if (m_pDev->Nunchuk.Acceleration.Orientation.UpdateAge < 10)
			{
				pdelta = iround((preview_frame.rows / 2) * tan((m_pDev->Acceleration.Orientation.Pitch)* M_PI / 180.0f));
				rdelta = iround((preview_frame.cols / 2) * tan((m_pDev->Acceleration.Orientation.Roll)* M_PI / 180.0f));
			}
			cv::line(preview_frame,
				cv::Point(0, preview_frame.rows / 2 + rdelta + pdelta),
				cv::Point(preview_frame.cols, preview_frame.rows / 2 - rdelta + pdelta),
				cv::Scalar(80, 80, 80),
				1);
		}

		//define a temp draw function
		auto fun = [&](const vec2& p, const cv::Scalar& color,int thinkness=1)
		{
			static constexpr int len = 9;

			cv::Point p2(iround(p[0] * preview_frame.cols + preview_frame.cols / 2),
				iround(-p[1] * preview_frame.cols + preview_frame.rows / 2));
			//cv::Point p2(iround(p[0]* preview_frame.cols/1024), iround(p[1] * preview_frame.rows / 768));
			cv::line(preview_frame,
				cv::Point(p2.x - len, p2.y),
				cv::Point(p2.x + len, p2.y),
				color,
				thinkness);
			cv::line(preview_frame,
				cv::Point(p2.x, p2.y - len),
				cv::Point(p2.x, p2.y + len),
				color,
				thinkness);
		};

		bool dot_sizes = (m_pDev->IR.Mode == wiimote_state::ir::EXTENDED);
		bool image_up = true;
		points.reserve(4);
		points.clear();

		for (unsigned index = 0; index < 4; index++)
		{
			wiimote_state::ir::dot &dot = m_pDev->IR.Dot[index];
			if (dot.bVisible) {
				//qDebug() << "wii:" << dot.RawX << "+" << dot.RawY;
				image_up = true;
				//vec2 dt((dot.RawX/1024.0f)-0.5f, ((-2.0f*dot.RawY)+768.0f)/(2.0f*1024.0f));
				//anti-clockwise rotate
				vec2 dt(((1024-dot.RawX) / 1024.0f) - 0.5f, ((-2.0f*(768-dot.RawY)) + 768.0f) / (2.0f*1024.0f));

				points.push_back(dt);
				if(dot_sizes)
					fun(dt, cv::Scalar(0, 255, 0), dot.Size);
				else
					fun(dt, cv::Scalar(0, 255, 0));
			}
		}
		const bool success = points.size() >= PointModel::N_POINTS;
		point_count = points.size();

#if 0
		CamInfo cam_info;
		cam_info.fps = 50;
		cam_info.res_x = 1024;
		cam_info.res_y = 768;
		cam_info.fov = 56;
#endif
		double fx;
		cam_info.get_focal_length(fx);
		if (success)
		{
			point_tracker.track(points,
				PointModel(s),
				cam_info,
				s.dynamic_pose ? s.init_phase_timeout : 0);
			ever_success = true;
			m_pDev->SetRumble(false);
			m_pDev->SetLEDs(0x0);
		}else {
			m_pDev->SetLEDs(4-point_count);
		}

		{
                Affine X_CM;
                {
                    QMutexLocker l(&data_mtx);
                    X_CM = point_tracker.pose();
                }

                // just copy pasted these lines from below
                Affine X_MH(mat33::eye(), vec3(s.t_MH_x, s.t_MH_y, s.t_MH_z));
                Affine X_GH = X_CM * X_MH;
                vec3 p = X_GH.t; // head (center?) position in global space
                vec2 p_((p[0] * fx) / p[2], (p[1] * fx) / p[2]);  // projected to screen

				fun(p_, cv::Scalar(0, 0, 255));
        }

		if(image_up)
			video_widget->update_image(preview_frame);

	}
	// Set event
goodbye:

	m_pDev->ChangedCallback = NULL;
	m_pDev->Disconnect();
	Beep(1000, 200);
	delete m_pDev;
	m_pDev = NULL;

	qDebug() << "FTNoIR_Tracker::run() terminated run()";
}



void Tracker_WII_PT::maybe_reopen_camera()
{
    QMutexLocker l(&camera_mtx);
#if 0
    Camera::open_status status = camera.start(camera_name_to_index(s.camera_name), s.cam_fps, s.cam_res_x, s.cam_res_y);

    switch (status)
    {
    case Camera::open_error:
        break;
    case Camera::open_ok_change:
#endif
        frame = cv::Mat();
#if 0
        break;
    case Camera::open_ok_no_change:
        break;
    }
#endif
}

void Tracker_WII_PT::set_fov(int value)
{
    QMutexLocker l(&camera_mtx);
    camera.set_fov(value);
}

void Tracker_WII_PT::start_tracker(QFrame* video_frame)
{
    //video_frame->setAttribute(Qt::WA_NativeWindow);
    preview_size = video_frame->size();

    preview_frame = cv::Mat(video_frame->height(), video_frame->width(), CV_8UC3);
    preview_frame.setTo(cv::Scalar(0, 0, 0));

    video_widget = qptr<cv_video_widget>(video_frame);
    layout = qptr<QHBoxLayout>(video_frame);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(video_widget.data());
    video_frame->setLayout(layout.data());
    //video_widget->resize(video_frame->width(), video_frame->height());
    video_frame->show();

    maybe_reopen_camera();

    start(QThread::HighPriority);
}

void Tracker_WII_PT::data(double *data)
{
    if (ever_success)
    {
        Affine X_CM = pose();

        Affine X_MH(mat33::eye(), vec3(s.t_MH_x, s.t_MH_y, s.t_MH_z));
        Affine X_GH = X_CM * X_MH;

        // translate rotation matrix from opengl (G) to roll-pitch-yaw (E) frame
        // -z -> x, y -> z, x -> -y
        mat33 R_EG(0, 0,-1,
                   -1, 0, 0,
                   0, 1, 0);
        mat33 R = R_EG *  X_GH.R * R_EG.t();

        // get translation(s)
        const vec3& t = X_GH.t;

        // extract rotation angles
        {
            f alpha, beta, gamma;
            beta  = atan2( -R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)) );
            alpha = atan2( R(1,0), R(0,0));
            gamma = atan2( R(2,1), R(2,2));

#if 0
            if (t[2] > 1e-4)
            {
                alpha += copysign(atan(t[0] / t[2]), t[0]);
                // pitch is skewed anyway due to only one focal length value
                //beta -= copysign(atan(t[1] / t[2]), t[1]);
            }
#endif

            data[Yaw] = rad2deg * alpha;
            data[Pitch] = -rad2deg * beta;
            data[Roll] = rad2deg * gamma;
        }

        // convert to cm
        data[TX] = t[0] / 10;
        data[TY] = t[1] / 10;
        data[TZ] = t[2] / 10;
    }
}

Affine Tracker_WII_PT::pose()
{
    QMutexLocker l(&data_mtx);

    return point_tracker.pose();
}

int Tracker_WII_PT::get_n_points()
{
    return int(point_count);
}

bool Tracker_WII_PT::get_cam_info(CamInfo* info)
{
    QMutexLocker lock(&camera_mtx);
    bool ret;

    std::tie(ret, *info) = camera.get_info();
    return ret;
}

#include "ftnoir_tracker_wii_pt_dialog.h"
OPENTRACK_DECLARE_TRACKER(Tracker_WII_PT, TrackerDialog_WII_PT, PT_metadata)

