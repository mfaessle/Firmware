#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <drivers/drv_gyro.h>

#include <systemlib/perf_counter.h>
#include <platforms/px4_defines.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gyro.h>

extern "C" __EXPORT int fw_manual_assist_main(int argc, char *argv[]);


class FixedWingManualAssistant
{
public:
  /**
   * Constructor
   */
  FixedWingManualAssistant();

  /**
   * Destructor, also kills the task.
   */
  ~FixedWingManualAssistant();

  /**
   * Start the task.
   *
   * @return            OK on success.
   */
  int start();

  bool task_running()
  {
    return _task_running;
  }

private:

  bool _task_should_exit; // if true, task should exit
  bool _task_running; // if true, task is running in its mainloop
  int _main_task; // task handle

  // subscribers
  int _gyro_sub;

  // publishers
  orb_advert_t _actuators_0_pub; // actuator control group 0 setpoint

  // data structs
  struct gyro_report _gyro_msg;
  struct actuator_controls_s _actuators; // actuator control inputs

  // performance counter
  perf_counter_t _loop_perf;

  // Shim for calling task_main from task_create.
  static void task_main_trampoline(int argc, char *argv[]);

  // Main task.
  void task_main();

  float limit(const float value, const float min, const float max);
};

namespace fw_manual_assist
{
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedWingManualAssistant *instance = nullptr;
}

FixedWingManualAssistant::FixedWingManualAssistant() :
    _task_should_exit(false),
    _task_running(false),
    _main_task(-1),

    /* subscriptions */
    _gyro_sub(-1),

    /* publishers*/
    _actuators_0_pub(nullptr),

    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "fw_manual_assist"))
{
  // Set memory of data structs
  _gyro_msg = {};
  _actuators = {};
}

FixedWingManualAssistant::~FixedWingManualAssistant()
{
  if (_main_task != -1)
  {
    /* task wakes up every 100ms or so at the longest */
    _task_should_exit = true;

    /* wait for a second for the task to quit at our request */
    unsigned i = 0;

    do
    {
      /* wait 20ms */
      usleep(20000);

      /* if we have given up, kill it */
      if (++i > 50)
      {
        px4_task_delete(_main_task);
        break;
      }
    } while (_main_task != -1);
  }

  perf_free(_loop_perf);

  fw_manual_assist::instance = nullptr;
}

void FixedWingManualAssistant::task_main_trampoline(int argc, char *argv[])
{
  fw_manual_assist::instance->task_main();
}

void FixedWingManualAssistant::task_main()
{
  warnx("fw_manual_assist started");

  // do subscriptions
  _gyro_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);

  // Set rate at which we subscribe
  orb_set_interval(_gyro_sub, 5); // 200 Hz

  // advertise published topics
  _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

  //  poll object
  px4_pollfd_struct_t fds[1];
  fds[0].fd = _gyro_sub;
  fds[0].events = POLLIN;

  _task_running = true;

  while (!_task_should_exit)
  {
    /* wait for up to 100ms for data */
    int ret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 100);

    /* timed out - periodic check for _task_should_exit */
    if (ret == 0)
      continue;

    /* this is undesirable but not much we can do - might want to flag unhappy status */
    if (ret < 0)
    {
      warn("poll error %d, %d", ret, errno);
      /* sleep a bit before next try */
      usleep(100000);
      continue;
    }

    perf_begin(_loop_perf);

    // run mainloop if we received data
    if (fds[0].revents & POLLIN)
    {
      // Copy gyro message
      orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &_gyro_msg);

      //printf("gyro: %5.5f  %5.5f  %5.5f\n", double(_gyro_msg.x), double(_gyro_msg.y), double(_gyro_msg.z));

      _actuators.control[0] = limit(-_gyro_msg.x / 10.0f, -1.0f, 1.0f);
      _actuators.control[1] = limit(-_gyro_msg.y / 10.0f, -1.0f, 1.0f);

      orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
    }

    perf_end(_loop_perf);
  }

  warnx("fw_manual_assist exit");

  _main_task = -1;
  _task_running = false;
  _exit(0);
}

float FixedWingManualAssistant::limit(const float value, const float min, const float max)
{
  float limited_value = value;
  if (limited_value < min)
  {
    limited_value = min;
  }
  else if (limited_value > max)
  {
    limited_value = max;
  }
  return limited_value;
}

int FixedWingManualAssistant::start()
{
  ASSERT(_main_task == -1);

  /* start the task */
  _main_task = px4_task_spawn_cmd("fw_manual_assist", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 1300,
                                 (px4_main_t)&FixedWingManualAssistant::task_main_trampoline, nullptr);

  if (_main_task < 0)
  {
    warn("task start failed");
    return -errno;
  }

  return OK;
}

int fw_manual_assist_main(int argc, char *argv[])
{
  if (argc < 2)
  {
    warnx("usage: fw_manual_assist {start|stop|status}");
    return 1;
  }

  if (!strcmp(argv[1], "start"))
  {

    if (fw_manual_assist::instance != nullptr)
    {
      warnx("already running");
      return 1;
    }

    fw_manual_assist::instance = new FixedWingManualAssistant;

    if (fw_manual_assist::instance == nullptr)
    {
      warnx("alloc failed");
      return 1;
    }

    if (OK != fw_manual_assist::instance->start())
    {
      delete fw_manual_assist::instance;
      fw_manual_assist::instance = nullptr;
      warn("start failed");
      return 1;
    }

    /* check if the waiting is necessary at all */
    if (fw_manual_assist::instance == nullptr || !fw_manual_assist::instance->task_running())
    {

      /* avoid memory fragmentation by not exiting start handler until the task has fully started */
      while (fw_manual_assist::instance == nullptr || !fw_manual_assist::instance->task_running())
      {
        usleep(50000);
        printf(".");
        fflush(stdout);
      }

      printf("\n");
    }

    return 0;
  }

  if (!strcmp(argv[1], "stop"))
  {
    if (fw_manual_assist::instance == nullptr)
    {
      warnx("not running");
      return 1;
    }

    delete fw_manual_assist::instance;
    fw_manual_assist::instance = nullptr;
    return 0;
  }

  if (!strcmp(argv[1], "status"))
  {
    if (fw_manual_assist::instance)
    {
      warnx("running");
      return 0;

    }
    else
    {
      warnx("not running");
      return 1;
    }
  }

  warnx("unrecognized command");
  return 1;
}
