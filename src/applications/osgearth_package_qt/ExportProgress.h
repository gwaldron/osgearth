#ifndef TILER_TOOL_EXPORT_PROGRESS_H
#define TILER_TOOL_EXPORT_PROGRESS_H 1

#include <osgEarth/Progress>
#include <QMainWindow>
#include <QMetaObject>
#include <QProgressDialog>

class ExportProgressCallback : public osgEarth::ProgressCallback
{
public:
    ExportProgressCallback(QMainWindow* parent, QProgressDialog* dialog);

    virtual ~ExportProgressCallback();

    bool reportProgress(double current, double total, unsigned currentStage, unsigned totalStages, const std::string& msg);

    void setStatus(const std::string& status);

    void complete();

private:
    QMainWindow* _parent;
    QProgressDialog* _dialog;    
    std::string _status;
};

#endif