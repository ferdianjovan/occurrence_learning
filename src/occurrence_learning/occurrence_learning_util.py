#!/usr/bin/env python

import math
import rospy
import calendar
import datetime
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


def week_of_month(tgtdate):
    """
        Assuming the first week of the month starts with the first monday of the month.
        If the month starts on any other day, then the first week will start from 0.
    """
    days_this_month = calendar.mdays[tgtdate.month]
    for i in range(1, days_this_month):
        d = datetime.date(tgtdate.year, tgtdate.month, i)
        if d.day - d.weekday() > 0:
            startdate = d
            break
    # now we can use the modulo 7 appraoch
    return (tgtdate - startdate).days // 7 + 1


def point_inside_polygon(x, y, poly):
    # credit to stackoverflow
    n = len(poly)
    inside = False

    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def time_ticks(minute_interval, window_interval, periodic_type="weekly"):
    """
        Get the text for weekly/monthly time ticks.
    """
    if periodic_type == "weekly":
        days = [
            "Monday", "Tuesday", "Wednesday", "Thursday", "Friday",
            "Saturday", "Sunday"
        ]
    else:
        days = [i+1 for i in range(31)]

    printed_hours = [0, 3, 6, 9, 12, 15, 18, 21]
    time_ticks = list()
    for j in days:
        for i in range(24 * (60/minute_interval)):
            hour = i / (60 / minute_interval)
            minute = (minute_interval * i) % 60
            if hour in printed_hours and minute == 0:
                if hour == 0:
                    time_ticks.append(
                        j+"\n"+datetime.time(hour, minute).isoformat()[:-3]
                    )
                else:
                    time_ticks.append(
                        datetime.time(hour, minute).isoformat()[:-3]
                    )
            else:
                time_ticks.append("")

    return time_ticks


def robot_view_cone(Px, Py, yaw):
    """
        let's call the triangle PLR, where P is the robot pose,
        L the left vertex, R the right vertex
    """
    d = 4       # max monitored distance: reasonably not more than 3.5-4m
    alpha = 1   # field of view: 57 deg kinect, 58 xtion, we can use exactly 1 rad (=57.3 deg)
    Lx = Px + d * (math.cos((yaw-alpha)/2))
    Ly = Py + d * (math.sin((yaw-alpha)/2))
    Rx = Px + d * (math.cos((yaw+alpha)/2))
    Ry = Py + d * (math.sin((yaw+alpha)/2))
    return [[Lx, Ly], [Rx, Ry], [Px, Py]]


def robot_view_area(Px, Py, yaw):
    d = 3
    poses = list()
    degree = 45 / float(180) * math.pi

    for i in range(8):
        x = Px + d * (math.cos((yaw+(i * degree) % (2 * math.pi))))
        y = Py + d * (math.sin((yaw+(i * degree) % (2 * math.pi))))
        poses.append([x, y])
    # return [[Px + 4, Py], [Px, Py - 4], [Px - 4, Py], [Px, Py + 4]]
    return poses


def trajectory_estimate_for_date(trajectory_estimate, date):
    """
        Get specific date from trajectory_estimate in the form {reg{date[hour{minute:traj}]}}.
        date is of the form datetime.date.
        It returns in the form {reg[hour{minute:traj}]}.
    """
    temp = dict()
    date = date.day
    for reg, daily_trajs in trajectory_estimate.iteritems():
        if reg not in temp:
            temp[reg] = list()
        if date in daily_trajs:
            temp[reg] = daily_trajs[date]
    return temp


def previous_n_minutes_trajs(prev_day_trajs, window_interval, minute_interval):
    """
       get the last (n-1) minutes window interval of trajectories.
    """
    temp_n_min_trajs = list()
    for i in range(
        60 - window_interval + minute_interval,
        60, minute_interval
    ):
        temp_n_min_trajs.append(prev_day_trajs[23][i])
    return temp_n_min_trajs


def _trajectories_full_dates_periodic(data, month, year, window_length, period_length, prev_day_n_min_traj=None):
    pointer = window_length - 1
    if prev_day_n_min_traj is None:
        temp_data = [-1 for i in range(window_length)]
    else:
        temp_data = prev_day_n_min_traj + [-1]

    total_daily_traj = dict()
    for date, hourly_trajs in data.iteritems():
        day = date
        if period_length == 7:
            day = datetime.date(year, month, date).weekday()
        if day not in total_daily_traj:
            total_daily_traj[day] = list()
        for hour, mins_traj in enumerate(hourly_trajs):
            mins = sorted(mins_traj)
            for i in mins:
                temp_data[pointer % window_length] = mins_traj[i]
                pointer += 1
                if sum(temp_data) == (-1 * window_length):
                    total_daily_traj[day].append(-1)
                else:
                    total_traj = window_length / float(window_length + sum([i for i in temp_data if i == -1]))
                    total_traj = math.ceil(total_traj * sum([i for i in temp_data if i != -1]))
                    total_daily_traj[day].append(total_traj)

    result = list()
    days = sorted(total_daily_traj)
    for i in days:
        result.extend(total_daily_traj[i])
    return result


def trajectories_full_dates_periodic(data, month, year, period_length, window_interval, minute_interval):
    """
        Obtain trajectories stored in the data of the form {day{minutes:total_traj}}.
        The result is a list of total trajectories within time window interval
        (window_interval).
        The dates of the data needs to be on a weekly period (capped at 7 days)
        or monthly period (capped at 31 days).
        For weekly period, the previous date from the original intention of dates must be
        given as well. E.g dates from 4-10, then the 3rd must be included (3-10).
        This does not apply if the intention dates include the first date of the month.
    """
    dates = sorted(data)
    delta = [dates[i]-dates[i-1] for i in range(1, len(dates))]
    result = list()
    if sum(delta) == len(delta):
        if len(dates) >= (period_length + 1):
            rospy.loginfo(
                "Obtaining total trajectories for dates %s month %d and year %d" % (
                    str(dates[1:period_length + 1]), month, year
                )
            )
            prev_day_n_min_traj = previous_n_minutes_trajs(
                data[dates[0]], window_interval, minute_interval
            )
            data = {i: data[i] for i in dates[1:period_length + 1]}
            result = _trajectories_full_dates_periodic(
                data, month, year, (window_interval/minute_interval),
                period_length, prev_day_n_min_traj
            )
        elif len(dates) == period_length:
            rospy.loginfo(
                "Obtaining total trajectories for dates %s month %d and year %d" % (
                    str(dates), month, year
                )
            )
            result = _trajectories_full_dates_periodic(
                data, month, year, (window_interval/minute_interval), period_length
            )
        else:
            rospy.logwarn(
                "The provided data does not have enough dates to fulfil %s days period" % period_length
            )
            rospy.logwarn("Returning empty result.")
    else:
        rospy.logwarn("The provided data does not have ordered dates or has skipped dates.")
        rospy.logwarn("Returning empty result.")
    return result


def rotation_180_quaternion(quaternion):
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    new_yaw = (yaw - math.pi) % (2 * math.pi)
    return quaternion_from_euler(roll, pitch, new_yaw)
