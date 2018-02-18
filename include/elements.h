#include <drawing.h>

namespace rcll_draw {

    struct Product {
        int complexity; //0-3
        int base;   // 1=RED, 2=BLACK, 3=SILVER
        int ring1;  // 1=BLUE, 2=GREEN, 3=ORANGE, 4=YELLOW
        int ring2;  // 1=BLUE, 2=GREEN, 3=ORANGE, 4=YELLOW
        int ring3;  // 1=BLUE, 2=GREEN, 3=ORANGE, 4=YELLOW
        int cap;    // 1=BLACK, 2=GREY

        int status_product; // 0=not started, 1=construction, 2=delivery, 3=completed
        int status_base;
        int status_ring1;
        int status_ring2;
        int status_ring3;
        int status_cap;
    };

    class HeaderPanel {
    public:
        HeaderPanel(std::string content, Team team);
        ~HeaderPanel();

        void setGeometry(int y, int w, int h);
        void draw(cv::Mat &mat);
    private:
        BoxLabel blbl_header;
    };

    // ##################################################

    class VStatusPanel {
    public:
        VStatusPanel();
        VStatusPanel(Team team);
        ~VStatusPanel();

        void setGeometry(int x, int y, int w, int h);
        void setContent(std::string gamestate, std::string gamephase, int time, int score);
        void draw(cv::Mat &mat);
    private:
        Team team;
        BoxLabel blbl_state_header;
        BoxLabel blbl_state_value;
        BoxLabel blbl_phase_header;
        BoxLabel blbl_phase_value;
        BoxLabel blbl_time_header;
        BoxLabel blbl_time_value;
        BoxLabel blbl_score_header;
        BoxLabel blbl_score_value;
    };

    // ##################################################

    class HStatusPanel {
    public:
        HStatusPanel();
        HStatusPanel(Team team);
        ~HStatusPanel();

        void setGeometry(int x, int y, int w, int h);
        void setContent(std::string gamestate, std::string gamephase, int time, int score);
        void draw(cv::Mat &mat);
    private:
        Team team;
        BoxLabel blbl_state_header;
        BoxLabel blbl_state_value;
        BoxLabel blbl_phase_header;
        BoxLabel blbl_phase_value;
        BoxLabel blbl_time_header;
        BoxLabel blbl_time_value;
        BoxLabel blbl_score_header;
        BoxLabel blbl_score_value;
    };

    // ##################################################

    class ProductLabel {
    public:
        ProductLabel();
        ~ProductLabel();

        void setGeometry(int x, int y, int w, int h);
        void setProduct(int id, Product plan, double blbl_progress, int blbl_deadline, int blbl_points, int points_max);
        cv::Mat createProductImage(Product plan);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_name;
        BoxLabel blbl_progress;
        BoxLabel blbl_deadline;
        BoxLabel blbl_points;
        Image product;
        std::vector<Image> img_step_progress;
        Image img_product_progress;
    };

    // ##################################################

    class ProductInfo {
    public:
        ProductInfo();
        ~ProductInfo();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index);
        void draw(cv::Mat &mat);

    private:
        std::vector<ProductLabel> products;
    };

    // ##################################################

    class MachineLabelProduction {
    public:
        MachineLabelProduction();
        ~MachineLabelProduction();

        void setGeometry(int x, int y, int w, int h);
        void setMachineName(std::string name);
        void setMachineStatus(std::string status, Color lamp_top, Color lamp_bottom, std::string lamp_top_str, std::string lamp_bottom_str);
        void setFlashing(bool flashing);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_border;
        BoxLabel blbl_machinename;
        BoxLabel blbl_state;
        Color lamp1_color;
        Color lamp2_color;
        BoxLabel blbl_lamp1;
        BoxLabel blbl_lamp2;
        bool flashing;
    };

    // ##################################################

    class MachineLabelExploration {
    public:
        MachineLabelExploration();
        ~MachineLabelExploration();

        void setGeometry(int x, int y, int w, int h);
        void setMachineName(std::string name, int index);
        void setMachineStatus(int status1, int status2);
        void setHeader(std::string status1, std::string status2);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_machinename;
        BoxLabel blbl_status1;
        BoxLabel blbl_status2;
        Image img_status1;
        Image img_status2;
    };

    // ##################################################

    class MachineInfoProduction {
    public:
        MachineInfoProduction();
        MachineInfoProduction(Team team);
        ~MachineInfoProduction();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setMachineName(std::string name, int index);
        void setMachineStatus(std::string status, int index);
        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header;
        std::vector<MachineLabelProduction> machines;
    };

    // ##################################################

    class MachineInfoExploration {
    public:
        MachineInfoExploration();
        MachineInfoExploration(Team team);
        ~MachineInfoExploration();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setMachineName(std::string name, int index);
        void setMachineStatus(int status1, int status2, int index);
        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header1;
        BoxLabel blbl_header2;
        std::vector<MachineLabelExploration> machines;
    };

    // ##################################################

    class RobotLabel {
    public:
        RobotLabel();
        ~RobotLabel();

        void setGeometry(int x, int y, int w, int h);
        void setRobotName(std::string name_str, bool active);
        void setRobotStatus(std::string mlblbl_activity, double active_time, int blbl_maintenance, int maintenance_max);
        void draw(cv::Mat &mat);

    private:
        BoxLabel blbl_name;
        MultilineBoxLabel mlblbl_activity;
        BoxLabel blbl_activetime;
        BoxLabel blbl_maintenance;
    };

    // ##################################################

    class RobotInfo {
    public:
        RobotInfo();
        RobotInfo(Team team);
        ~RobotInfo();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setRobotName(std::string name_str, bool active, int index);
        void setRobotStatus(std::string activity, double active_time, int maintenance_count, int maintenance_max, int index);
        void draw(cv::Mat &mat);

    private:
        Team team;
        BoxLabel blbl_header;
        std::vector<RobotLabel> robots;
    };

    // ##################################################

    class TeamAreaProduction {
    public:
        TeamAreaProduction(Team team);
        ~TeamAreaProduction();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setGameInfo(std::string gamestate, std::string gamephase, int time, int score);
        void setMachineName(std::string name, int index);
        void setMachineStatus(std::string status, int index);
        void setRobotName(std::string name, bool active, int index);
        void setRobotStatus(std::string activity, double active_time, int maintenance_count, int maintenance_max, int index);
        void setProduct(int id, Product plan, double progress, int deadline, int points, int points_max, int index);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        Team team;
        VStatusPanel game_info;
        ProductInfo product_info;
        MachineInfoProduction machine_info;
        RobotInfo robot_info;


    };

    // ##################################################

    class TeamAreaExploration {
    public:
        TeamAreaExploration(Team team);
        ~TeamAreaExploration();

        void setGeometry(int x, int y, int w, int h, int gapsize);
        void setGameInfo(std::string gamestate, std::string gamephase, int time, int score);
        void setMachineName(std::string name, int index);
        void setMachineStatus(int status1, int status2, int index);
        void draw(cv::Mat &mat);

    private:
        int x, y = 0;
        int w, h = 1;

        Team team;
        HStatusPanel game_info;
        MachineInfoExploration machine_info;


    };
}
