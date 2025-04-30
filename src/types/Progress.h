#pragma once

#include <string>

namespace meshlib {

class Progress {
public:
    Progress() {}
    virtual ~Progress() {}
    virtual void newSection(const std::string& ) {}
    virtual void endSection() {}
    virtual void newTask(const std::string&, const std::size_t&) {}
    virtual bool advanceTask(const std::size_t & =1) { return false; }
    virtual void endTask() {};

private:
    Progress(const Progress&);
    Progress& operator=(const Progress&);
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
    void newSection(const std::string& name) const {
        progress_->newSection(name);
    };
    void endSection() const {
        progress_->endSection();
    };
    void newTask(const std::string& name, const std::size_t& size) const {
        progress_->newTask(name, size);
    };
    bool advanceTask(const std::size_t& size = 1) const {
        return progress_->advanceTask(size);
    };
    void endTask() const {
        progress_->endTask();
    };
private:
    bool assign_;
    Progress* progress_;
    Progress  emptyProgress_;
};

}

