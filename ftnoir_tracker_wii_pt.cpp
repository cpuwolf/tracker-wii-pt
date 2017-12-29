/* Copyright (c) 2012 Patrick Ruoff
 * Copyright (c) 2014-2016 Stanislaw Halik <sthalik@misaki.pl>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#include "ftnoir_tracker_wii_pt.h"
#include "compat/camera-names.hpp"
#include <QHBoxLayout>
#include <cmath>
#include <QDebug>
#include <QFile>
#include <QCoreApplication>
#include <functional>

//#define PT_PERF_LOG	//log performance

//-----------------------------------------------------------------------------
Tracker_WII_PT::Tracker_WII_PT() :
      point_count(0),
      commands(0),
      ever_success(false)
{
    connect(s.b.get(), SIGNAL(saving()), this, SLOT(apply_settings()), Qt::DirectConnection);
}

Tracker_WII_PT::~Tracker_WII_PT()
{
    set_command(ABORT);
    wait();

    QMutexLocker l(&camera_mtx);
    camera.stop();
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
reconnect:
	qDebug() << "wii wait";
	while (!m_pDev->Connect(wiimote::FIRST_AVAILABLE)) {
		if (commands & ABORT)
			goto goodbye;
		Beep(500, 30); Sleep(1000);
	}

	/* wiimote connected */
	m_pDev->SetLEDs(0x0f);
	Beep(1000, 300); Sleep(1000);

	m_pDev->SetLEDs(0x01);
	m_pDev->SetRumble(true);
	Sleep(1);
	m_pDev->SetRumble(false);
	

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
#if 0
		points.clear();
		for (unsigned index = 0; index < 4; index++)
		{
			wiimote_state::ir::dot &dot = m_pDev->IR.Dot[index];
			if (dot.bVisible) {
				//qDebug() << "wii:" << dot.RawX << "+" << dot.RawY;
				vec2 dt(dot.RawX/1024.0, dot.RawY/768.0);
				//vec2 dt(10.0, 10.0);
				points.push_back(dt);
			}
		}

#endif	
		CamInfo cam_info;
		camera.get_frame(1.0, frame, cam_info);

		//create preview video frame
		cv::resize(blank_frame, preview_frame, cv::Size(preview_size.width(), preview_size.height()), 0, 0, cv::INTER_NEAREST);

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
		bool image_up = false;
		points.reserve(4);
		points.clear();

		for (unsigned index = 0; index < 4; index++)
		{
			wiimote_state::ir::dot &dot = m_pDev->IR.Dot[index];
			if (dot.bVisible) {
				//qDebug() << "wii:" << dot.RawX << "+" << dot.RawY;
				image_up = true;
				//vec2 dt(dot.RawX, dot.RawY);
				vec2 dt((dot.RawX/1024.0f)-0.5f, ((-2.0f*dot.RawY)+768.0f)/(2.0f*1024.0f));
				//vec2 dt((dot.RawX / 1024.0f) - 0.5f, (dot.RawY / 768.0f) - 0.5f);

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
		if (success)
		{
			point_tracker.track(points,
				PointModel(s),
				cam_info,
				s.dynamic_pose ? s.init_phase_timeout : 0);
			ever_success = true;
		}
		if(image_up)
			video_widget->update_image(preview_frame);

		//if(m_pDev->Nunchuk.Acceleration.Orientation.UpdateAge > 10)
		//{
		//--newHeadPose.pitch = m_pDev->Acceleration.Orientation.Pitch;
		//--newHeadPose.roll = m_pDev->Acceleration.Orientation.Roll;
		//printf("pitch %f roll %f yaw %f\n",newHeadPose.pitch, newHeadPose.roll, newHeadPose.yaw);
		//QMessageBox::warning(0,"HeadTrack Error", "pitch and roll",QMessageBox::Ok,QMessageBox::NoButton);
		//}
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

void Tracker_WII_PT::runold()
{
    cv::setNumThreads(0);

#ifdef PT_PERF_LOG
    QFile log_file(QCoreApplication::applicationDirPath() + "/PointTrackerPerformance.txt");
    if (!log_file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream log_stream(&log_file);
#endif

    apply_settings();

    while((commands & ABORT) == 0)
    {
        const double dt = time.elapsed_seconds();
        time.start();
        CamInfo cam_info;
        bool new_frame;

        {
            QMutexLocker l(&camera_mtx);
            new_frame = camera.get_frame(dt, frame, cam_info);
        }

        cv::resize(frame, preview_frame, cv::Size(preview_size.width(), preview_size.height()), 0, 0, cv::INTER_NEAREST);

        if (new_frame && !frame.empty())
        {
            point_extractor.extract_points(frame, preview_frame, points);
            point_count = points.size();

            f fx;
            cam_info.get_focal_length(fx);

            const bool success = points.size() >= PointModel::N_POINTS;

            if (success)
            {
                point_tracker.track(points,
                                    PointModel(s),
                                    cam_info,
                                    s.dynamic_pose ? s.init_phase_timeout : 0);
                ever_success = true;
            }

            auto fun = [&](const vec2& p, const cv::Scalar& color)
            {
                static constexpr int len = 9;

                cv::Point p2(iround(p[0] * preview_frame.cols + preview_frame.cols/2),
                             iround(-p[1] * preview_frame.cols + preview_frame.rows/2));
                cv::line(preview_frame,
                         cv::Point(p2.x - len, p2.y),
                         cv::Point(p2.x + len, p2.y),
                         color,
                         1);
                cv::line(preview_frame,
                         cv::Point(p2.x, p2.y - len),
                         cv::Point(p2.x, p2.y + len),
                         color,
                         1);
            };

            for (unsigned i = 0; i < points.size(); i++)
            {
                fun(points[i], cv::Scalar(0, 255, 0));
            }

            {
                Affine X_CM;
                {
                    QMutexLocker l(&data_mtx);
                    X_CM = point_tracker.pose();
                }

                Affine X_MH(mat33::eye(), vec3(s.t_MH_x, s.t_MH_y, s.t_MH_z)); // just copy pasted these lines from below
                Affine X_GH = X_CM * X_MH;
                vec3 p = X_GH.t; // head (center?) position in global space
                vec2 p_(p[0] / p[2] * fx, p[1] / p[2] * fx);  // projected to screen
                fun(p_, cv::Scalar(0, 0, 255));
            }

			

            video_widget->update_image(preview_frame);
        }
    }
    qDebug() << "pt: thread stopped";
}

void Tracker_WII_PT::apply_settings()
{
    qDebug() << "pt: applying settings";

    QMutexLocker l(&camera_mtx);

    CamInfo info;

    if (!camera.get_info(info) || frame.rows != info.res_y || frame.cols != info.res_x)
        frame = cv::Mat();

    if (!camera.start(camera_name_to_index(s.camera_name), s.cam_fps, s.cam_res_x, s.cam_res_y))
        qDebug() << "can't start camera" << s.camera_name;

    qDebug() << "pt: done applying settings";
}

void Tracker_WII_PT::start_tracker(QFrame* video_frame)
{
    video_frame->setAttribute(Qt::WA_NativeWindow);
    preview_size = video_frame->size();
    video_widget = qptr<wiiv_video_widget>(video_frame);
    layout = qptr<QHBoxLayout>(video_frame);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(video_widget.data());
    video_frame->setLayout(layout.data());
    //video_widget->resize(video_frame->width(), video_frame->height());
    video_frame->show();
    start();
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

        using std::atan2;
        using std::sqrt;
        using std::atan;
        using std::fabs;
        using std::copysign;

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

    return camera.get_info(*info);
}

#include "ftnoir_tracker_wii_pt_dialog.h"
OPENTRACK_DECLARE_TRACKER(Tracker_WII_PT, TrackerDialog_WII_PT, PT_metadata)

