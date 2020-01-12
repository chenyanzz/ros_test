#pragma once

#include <QtWidgets/QWidget>
#include "ui_QDashboard.h"
#include "LineItemWidget.h"
#include <qtimer.h>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

class QDashboard : public QWidget
{
	Q_OBJECT

protected:
	QTimer timer;
	
public:
	QDashboard(QWidget *parent = Q_NULLPTR)
		: QWidget(parent)
	{
		ui.setupUi(this);
	}

	void addItem(LineItemWidgetBase *item)
	{
		ui.Contents->addLayout(item);
	}

	bool hasItem(QString name)
	{
		return LineItemWidgetBase::getItemMap().contains(name);
	}

	template <class ItemType>
	ItemType *getItem(QString name)
	{
		if (!hasItem(name))
			return nullptr;
		LineItemWidgetBase *item = LineItemWidgetBase::getItemMap()[name];
		if (0 != strcmp(ItemType::staticMetaObject.className(), item->metaObject()->className()))
			return nullptr;
		return reinterpret_cast<ItemType *>(item);
	}

private:
	Ui::QDashboardClass ui;
};