#pragma once

#include <string>
#include <vector>

namespace meshlib {

class Progress {
public:
    Progress() {}
    virtual ~Progress() {}
    virtual void setSections(const std::vector<double>& /*section_weights*/){}
    virtual void newSection(const std::string&,const int /*section_num_tasks*/=0) {}
    virtual void endSection() {}
    virtual void newTask(const std::string&,const size_t /*task_num_steps*/=1) {}
    virtual bool advanceTask() { return false; }
    virtual bool endTask() { return false; }

private:
    Progress(const Progress&);
};

class ProgressManager {
public:
    ProgressManager() {
        assign_ = false;
        progress_ = &emptyProgress_;
    }
    ProgressManager(const ProgressManager& rhs) {
        assign_ = rhs.assign_;
        if (assign_) {
            progress_ = rhs.progress_;
        } else {
            progress_ = &emptyProgress_;
        }
    }
    virtual ~ProgressManager() {

    }
    void operator=(Progress* progress) {
        assign_ = true;
        progress_ = progress;
    }
    operator Progress&() const {
        return *progress_;
    }
    void setSections(const std::vector<double>& section_weights){
        progress_->setSections(section_weights);
    }
    void newSection(const std::string& name,const int section_num_tasks=0) const {
        progress_->newSection(name,section_num_tasks);
    };
    void endSection() const {
        progress_->endSection();
    };
    void newTask(const std::string& name,const size_t task_num_steps=1) const {
        progress_->newTask(name,task_num_steps);
    };
    bool advanceTask() const {
        return progress_->advanceTask();
    };
    bool endTask() const {
        return progress_->endTask();
    };
private:
    bool assign_;
    Progress* progress_;
    Progress  emptyProgress_;
};

}

