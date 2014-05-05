#include "ExportProgress.h"

#include <sstream>

ExportProgressCallback::ExportProgressCallback(QMainWindow* parent, QProgressDialog* dialog)
    : _parent(parent), _dialog(dialog)
{
}

ExportProgressCallback::~ExportProgressCallback()
{
}

bool ExportProgressCallback::reportProgress(double current, double total, unsigned currentStage, unsigned totalStages, const std::string& msg)
{
    if (_dialog)
    {
        int percentComplete = (current / total) * 100;                
        QMetaObject::invokeMethod(_dialog, "setValue", Qt::QueuedConnection, Q_ARG(int, percentComplete));        

        // Update the status label
        std::stringstream buf;
        buf << _status << "\n" << (int)current << " of " << (int)total;        
        QString qstatus = QString::fromStdString(buf.str());
        QMetaObject::invokeMethod(_dialog, "setLabelText", Qt::BlockingQueuedConnection, Q_ARG(const QString&, qstatus) );        

        if (_dialog->wasCanceled())
        {
            return true;
        }
    }

    return false;
}

void ExportProgressCallback::setStatus(const std::string& status)
{    
    _status = status;
}


void ExportProgressCallback::complete()
{    
    bool userCanceled = _dialog->wasCanceled();
    QMetaObject::invokeMethod(_dialog, "close", Qt::QueuedConnection);

    if (!userCanceled)
    QMetaObject::invokeMethod(_parent, "showExportResult", Qt::BlockingQueuedConnection);    
}
