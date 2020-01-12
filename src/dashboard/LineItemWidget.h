#pragma once

#include <QHBoxLayout>
#include <QtWidgets/qwidget.h>
#include <QtWidgets/qlabel.h>
#include <QtWidgets/qlineedit.h>
#include <QtWidgets/qslider.h>
#include <QtWidgets/qcheckbox.h>
#include <QMap>
#include <ros/ros.h>


class LineItemWidgetBase : public QHBoxLayout
{
	Q_OBJECT

public:
	typedef QMap<QString, LineItemWidgetBase *> ItemMap;

private:
	QLabel *label = nullptr;
	QWidget *dataWidget = nullptr;
	static ItemMap item_map;

public:
	const QString title;
	const bool editable;
	const QString type;
	
	static const ItemMap &getItemMap() { return item_map; }

	LineItemWidgetBase(const QString& _type,const QString& _title, bool _editable)
		: title(_title), editable(_editable), type(_type)
	{
		this->label = new QLabel(title + ":\t");
		this->addWidget(label);

		item_map.insert(title, this);
	}

	LineItemWidgetBase(const QString& type,const QString& _title, bool editable, QWidget *_dataWidget)
		: LineItemWidgetBase(type, _title, editable)
	{
		setDataWidget(_dataWidget);
	}

	~LineItemWidgetBase()
	{
		item_map.remove(title);
		delete dataWidget;
		delete label;
	}

	void setDataWidget(QWidget *_dataWidget)
	{
		Q_ASSERT(_dataWidget != nullptr);

		if (dataWidget)
			this->removeWidget(dataWidget);
		dataWidget = _dataWidget;
		this->addWidget(dataWidget);
	}
};

class StringDataItem : public LineItemWidgetBase
{
	Q_OBJECT

protected:
	QLineEdit *dataLineEdit = nullptr;

public:
	StringDataItem(QString title, QString data = "", bool isEditable = false) : LineItemWidgetBase(QString("string"), title, isEditable)
	{
		dataLineEdit = new QLineEdit();
		dataLineEdit->setReadOnly(!isEditable);
		this->setDataWidget(dataLineEdit);
		this->setData(data);
	}

	void setData(QString data)
	{
		Q_ASSERT(dataLineEdit != nullptr);
		if(getData()!=data)
			dataLineEdit->setText(data);
	}

	QString getData()
	{
		Q_ASSERT(dataLineEdit != nullptr);
		return dataLineEdit->text();
	}
};

class NumberDataItem : public LineItemWidgetBase
{
	Q_OBJECT

protected:
	QSlider *numberSlider = nullptr;
	QLabel *numberLabel = nullptr;

public:
	NumberDataItem(QString title, int val = 0, bool editable = true, int min = 0, int max = 100, int step = 1) : LineItemWidgetBase("number", title, editable)
	{
		numberSlider = new QSlider(Qt::Horizontal);
		numberLabel = new QLabel();
		numberLabel->setNum(val);
		numberSlider->setRange(min, max);
		numberSlider->setSingleStep(step);
		numberSlider->setAcceptDrops(editable);
		this->addWidget(numberLabel);
		this->setDataWidget(numberSlider);
		setValue(val);
		connect(numberSlider, SIGNAL(valueChanged(int)), numberLabel, SLOT(setNum(int)));
	}

	~NumberDataItem() { delete numberLabel; }

	void setValue(int val)
	{
		Q_ASSERT(numberSlider != nullptr);
		if(getValue()!=val)
			numberSlider->setValue(val);
	}

	int getValue()
	{
		Q_ASSERT(numberSlider != nullptr);
		return numberSlider->value();
	}
};

class BoolDataItem : public LineItemWidgetBase
{
	Q_OBJECT

protected:
	QCheckBox *checkbox = nullptr;

public:
	BoolDataItem(QString title, bool state = false, bool editable = true) : LineItemWidgetBase("bool", title, editable)
	{
		checkbox = new QCheckBox;
		checkbox->setCheckable(editable);
		this->setDataWidget(checkbox);
	}

	void setState(bool val)
	{
		Q_ASSERT(checkbox != nullptr);
		if(getState()!=val)
			checkbox->setChecked(val);
	}

	bool getState()
	{
		Q_ASSERT(checkbox != nullptr);
		return checkbox->isChecked();
	}
};