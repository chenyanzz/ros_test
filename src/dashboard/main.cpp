#include "QDashboard.h"
#include <QtWidgets/QApplication>
#include "LineItemWidget.h"
#include <qtimer.h>
#include <qlist.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <dashboard/Request.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <QMap>
#include <QProcess>

using namespace std;

void subscribeTopics(ros::NodeHandle& node);
void broadcastTopics();
void dataRequestHandler(const dashboard::Request &req);
QString topicsToStr();

QDashboard *dash = nullptr;
QMap<QString, ros::Publisher> pubMap;
QMap<QString, ros::Subscriber> subMap;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dashboard_node");
	ros::NodeHandle node;

	QApplication a(argc, argv);
	dash = new QDashboard(nullptr);
	dash->show();

	QTimer rosRefreshTimer;
	rosRefreshTimer.setInterval(200);

	a.connect(&rosRefreshTimer, &QTimer::timeout, [&]() {
		ros::spinOnce();
		if (!ros::ok())
			a.exit(0);
		subscribeTopics(node);
		broadcastTopics();
		ros::spinOnce();
	});

	rosRefreshTimer.start();

	return a.exec();
}

bool startsWith(const string &s, const string &sub)
{
	return s.find(sub) == 0;
}

bool toBool(const QString &s)
{
	return s == "true" || s == "True" || s == "1";
}

void dataRequestHandler(const dashboard::Request &req)
{
	static ros::NodeHandle node;
	auto data_name = QString::fromStdString(req.name);
	auto data_str = QString::fromStdString(req.data);
	const auto &item_map = LineItemWidgetBase::getItemMap();

	if (req.action == "remove")
	{
		if (item_map.contains(data_name))
		{
			delete item_map[data_name];
		}
		if (pubMap.contains(data_name))
		{
			pubMap.remove(data_name);
		}
		if (subMap.contains(data_name))
		{
			subMap.remove(data_name);
		}
	}
	else if (req.action == "set")
	{
		if (req.type == "number")
		{
			auto sl = data_str.split("|");

			int val = sl.size() >= 1 ? sl[0].toInt() : 0;
			int min = sl.size() >= 2 ? sl[1].toInt() : 0;
			int max = sl.size() >= 3 ? sl[2].toInt() : 100;
			int step = sl.size() >= 4 ? sl[3].toInt() : 1;

			if (dash->hasItem(data_name))
			{
				auto *item = dash->getItem<NumberDataItem>(data_name);
				if (item != nullptr)
					item->setValue(val);
			}
			else
			{
				dash->addItem(new NumberDataItem(data_name, val, req.editable, min, max, step));
				pubMap.insert(data_name, node.advertise<std_msgs::Int32>((std::string("dashboard_reply/") + data_name.toStdString()).c_str(), 10));
			}
		}
		else if (req.type == "bool")
		{
			bool state = toBool(data_str);
			if (dash->hasItem(data_name))
			{
				auto *item = dash->getItem<BoolDataItem>(data_name);
				if (item != nullptr)
					item->setState(state);
			}
			else
			{
				dash->addItem(new BoolDataItem(data_name, state, req.editable));
				pubMap.insert(data_name, node.advertise<std_msgs::Bool>((std::string("dashboard_reply/") + data_name.toStdString()).c_str(), 10));
			}
		}
		else if (req.type == "string")
		{
			if (dash->hasItem(data_name))
			{
				auto *item = dash->getItem<StringDataItem>(data_name);
				if (item != nullptr)
					item->setData(data_str);
			}
			else
			{
				dash->addItem(new StringDataItem(data_name, data_str, req.editable));
				pubMap.insert(data_name, node.advertise<std_msgs::String>((std::string("dashboard_reply/") + data_name.toStdString()).c_str(), 10));
			}
		}
		else
		{
			ROS_WARN("UNKNOWN request type: %s", req.type.c_str());
			return;
		}
	}
	else if (req.action == "clear")
	{
		for (auto &item : item_map)
		{
			delete item;
		}
		pubMap.clear();
		subMap.clear();
	}
}

QString executeLinuxCmd(QString strCmd)
{
	QProcess p;
	p.start("bash", QStringList() << "-c" << strCmd);
	
	while(!p.waitForFinished(10)){
		QApplication::processEvents();
		ros::spinOnce();
	}

	QString strResult = p.readAllStandardOutput();
	return strResult;
}

void subscribeTopics(ros::NodeHandle &node)
{
	static const QString topic_folder = "dashboard";

	auto sl = executeLinuxCmd("rostopic list -v").split('\n');

	QRegExp patternExp(R"(.*/(dashboard/.*) \[(.*)\].*)");

	for (auto line : sl.filter("*").filter("publisher"))
	{
		std::string l = line.toStdString();
		if (!patternExp.exactMatch(line))
			continue;

		auto topic_name = patternExp.cap(1);
		auto topic_type = patternExp.cap(2);

		//ROS_WARN("topic: %s", topic_name.toStdString().c_str());
		auto sl = topic_name.split("/", QString::SkipEmptyParts);

		//return if met illegal topic
		if (sl.size() < 2)
			return;
		if (sl[0] != topic_folder)
			return;
		if (topic_type != "dashboard/Request")
			return;

		auto &item_name = sl[1];

		ROS_INFO("getDataTopic: %s", topic_name.toStdString().c_str());

		if (LineItemWidgetBase::getItemMap().contains(item_name) || subMap.contains(item_name))
			return;

		auto sub = node.subscribe(topic_name.toStdString(), 100, &dataRequestHandler);
		subMap.insert(item_name, sub);
	}
}

void broadcastTopics()
{
	for (auto item : LineItemWidgetBase::getItemMap())
	{
		if (!item->editable)
			continue;
		if (item->type == "string")
		{
			std_msgs::String msg;
			msg.data = reinterpret_cast<StringDataItem *>(item)->getData().toStdString();
			pubMap[item->title].publish(msg);
		}
		else if (item->type == "number")
		{
			std_msgs::Int32 msg;
			msg.data = reinterpret_cast<NumberDataItem *>(item)->getValue();
			pubMap[item->title].publish(msg);
		}
		else if (item->type == "bool")
		{
			std_msgs::Bool msg;
			msg.data = reinterpret_cast<BoolDataItem *>(item)->getState();
			pubMap[item->title].publish(msg);
		}
	}
}

QString topicsToStr()
{
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);
	QString s;
	for (auto topic : topics)
	{
		s += QString::fromStdString(topic.name);
		s += "[";
		s += QString::fromStdString(topic.datatype);
		s += "] | ";
	}
	return s;
}