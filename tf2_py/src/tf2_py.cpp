#include <Python.h>

#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>

#include <memory>
#include <new>
#include <string>
#include <vector>

/// \brief Converts a C++ string into a Python string.
/// \note The caller is responsible for decref'ing the returned object.
/// \note If the return value is NULL then an exception is set.
/// \return a new PyObject reference, or NULL
inline PyObject * stringToPython(const std::string & input)
{
  return PyUnicode_FromStringAndSize(input.c_str(), input.size());
}

/// \brief Converts a C string into a Python string.
/// \note The caller is responsible for decref'ing the returned object.
/// \note If the return value is NULL then an exception is set.
/// \return a new PyObject reference, or NULL
inline PyObject * stringToPython(const char * input)
{
  return PyUnicode_FromString(input);
}

/// \brief Converts a Python string into a C++ string.
/// \note The input PyObject is borrowed, and will not be decref'd.
/// \note It's possible for this function to set an exception.
///   If the returned string is empty, callers should check if an exception was
///   set using PyErr_Occurred().
/// \return a new std::string instance
inline std::string stringFromPython(PyObject * input)
{
  Py_ssize_t size;
  const char * data = PyUnicode_AsUTF8AndSize(input, &size);
  return data ? std::string(data, size) : std::string();
}

/// \brief Imports a python module by name.
/// \note The caller is responsible for decref'ing the returned object.
/// \note If the return value is NULL then an exception is set.
/// \return a reference to the imported module.
static PyObject * importAttribute(const char * module_name, const char * attribute_name)
{
  PyObject * module = PyImport_ImportModule(module_name);
  if (!module) {
    return nullptr;
  }
  PyObject * attribute = PyObject_GetAttrString(module, attribute_name);
  Py_DECREF(module);
  return attribute;
}

struct TfModuleState
{
  PyObject * transform_exception;
  PyObject * connectivity_exception;
  PyObject * lookup_exception;
  PyObject * extrapolation_exception;
  PyObject * invalid_argument_exception;
  PyObject * timeout_exception;
};

extern PyModuleDef tf_module;

static TfModuleState * moduleState(PyObject * self)
{
  PyObject * module = PyType_GetModuleByDef(Py_TYPE(self), &tf_module);
  return module ? static_cast<TfModuleState *>(PyModule_GetState(module)) : nullptr;
}

// Run x (a tf method, catching TF's exceptions and reraising them as Python exceptions)
//
#define WRAP(x) \
  do { \
    const auto * state = moduleState(self); \
    if (!state) { \
      return nullptr; \
    } \
    try \
    { \
      x; \
    } \
    catch (const tf2::ConnectivityException & e) \
    { \
      PyErr_SetString(state->connectivity_exception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::LookupException & e) \
    { \
      PyErr_SetString(state->lookup_exception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::ExtrapolationException & e) \
    { \
      PyErr_SetString(state->extrapolation_exception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::InvalidArgumentException & e) \
    { \
      PyErr_SetString(state->invalid_argument_exception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::TimeoutException & e) \
    { \
      PyErr_SetString(state->timeout_exception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::TransformException & e) \
    { \
      PyErr_SetString(state->transform_exception, e.what()); \
      return nullptr; \
    } \
  } while (0)

struct buffer_core_t
{
  PyObject_HEAD
  std::shared_ptr<tf2::BufferCore> bc;
};

static std::shared_ptr<tf2::BufferCore> getBufferCore(PyObject * self)
{
  auto bc = std::atomic_load(&reinterpret_cast<buffer_core_t *>(self)->bc);
  if (!bc) {
    PyErr_SetString(PyExc_RuntimeError, "BufferCore has not been initialized");
  }
  return bc;
}

static PyObject * transform_converter(const geometry_msgs::msg::TransformStamped * transform)
{
  PyObject * pclass = nullptr;
  PyObject * pargs = nullptr;
  PyObject * pinst = nullptr;
  PyObject * builtin_interfaces_time = nullptr;
  PyObject * args = nullptr;
  PyObject * kwargs = nullptr;
  PyObject * sec = nullptr;
  PyObject * nanosec = nullptr;
  PyObject * time_obj = nullptr;
  PyObject * pheader = nullptr;
  PyObject * pframe_id = nullptr;
  PyObject * ptransform = nullptr;
  PyObject * ptranslation = nullptr;
  PyObject * protation = nullptr;
  PyObject * child_frame_id = nullptr;
  PyObject * ptrans_x = nullptr;
  PyObject * ptrans_y = nullptr;
  PyObject * ptrans_z = nullptr;
  PyObject * prot_x = nullptr;
  PyObject * prot_y = nullptr;
  PyObject * prot_z = nullptr;
  PyObject * prot_w = nullptr;
  pclass = importAttribute("geometry_msgs.msg", "TransformStamped");
  if (!pclass) {
    goto cleanup;
  }

  pargs = Py_BuildValue("()");
  if (!pargs) {
    goto cleanup;
  }

  pinst = PyObject_Call(pclass, pargs, NULL);
  if (!pinst) {
    goto cleanup;
  }

  // we need to convert the time to python
  builtin_interfaces_time = importAttribute("builtin_interfaces.msg", "Time");
  if (!builtin_interfaces_time) {
    goto cleanup;
  }
  args = PyTuple_New(0);
  if (!args) {
    goto cleanup;
  }
  kwargs = PyDict_New();
  if (!kwargs) {
    goto cleanup;
  }
  sec = Py_BuildValue("i", transform->header.stamp.sec);
  if (!sec) {
    goto cleanup;
  }
  nanosec = Py_BuildValue("i", transform->header.stamp.nanosec);
  if (!nanosec) {
    goto cleanup;
  }
  if (-1 == PyDict_SetItemString(kwargs, "sec", sec)) {
    goto cleanup;
  }
  if (-1 == PyDict_SetItemString(kwargs, "nanosec", nanosec)) {
    goto cleanup;
  }

  time_obj = PyObject_Call(builtin_interfaces_time, args, kwargs);
  if (!time_obj) {
    goto cleanup;
  }

  pheader = PyObject_GetAttrString(pinst, "header");
  if (!pheader) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(pheader, "stamp", time_obj)) {
    goto cleanup;
  }

  pframe_id = stringToPython(transform->header.frame_id);
  if (!pframe_id) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(pheader, "frame_id", pframe_id)) {
    goto cleanup;
  }

  ptransform = PyObject_GetAttrString(pinst, "transform");
  if (!ptransform) {
    goto cleanup;
  }
  ptranslation = PyObject_GetAttrString(ptransform, "translation");
  if (!ptranslation) {
    goto cleanup;
  }
  protation = PyObject_GetAttrString(ptransform, "rotation");
  if (!ptranslation) {
    goto cleanup;
  }

  child_frame_id = stringToPython(transform->child_frame_id);
  if (!child_frame_id) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(pinst, "child_frame_id", child_frame_id)) {
    goto cleanup;
  }

  ptrans_x = PyFloat_FromDouble(transform->transform.translation.x);
  if (!ptrans_x) {
    goto cleanup;
  }
  ptrans_y = PyFloat_FromDouble(transform->transform.translation.y);
  if (!ptrans_y) {
    goto cleanup;
  }
  ptrans_z = PyFloat_FromDouble(transform->transform.translation.z);
  if (!ptrans_z) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(ptranslation, "x", ptrans_x)) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(ptranslation, "y", ptrans_y)) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(ptranslation, "z", ptrans_z)) {
    goto cleanup;
  }

  prot_x = PyFloat_FromDouble(transform->transform.rotation.x);
  if (!prot_x) {
    goto cleanup;
  }

  prot_y = PyFloat_FromDouble(transform->transform.rotation.y);
  if (!prot_y) {
    goto cleanup;
  }

  prot_z = PyFloat_FromDouble(transform->transform.rotation.z);
  if (!prot_z) {
    goto cleanup;
  }

  prot_w = PyFloat_FromDouble(transform->transform.rotation.w);
  if (!prot_w) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "x", prot_x)) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "y", prot_y)) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "z", prot_z)) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "w", prot_w)) {
    goto cleanup;
  }
cleanup:
  if (PyErr_Occurred()) {
    Py_XDECREF(pinst);
    pinst = nullptr;
  }
  Py_XDECREF(pclass);
  Py_XDECREF(pargs);
  Py_XDECREF(builtin_interfaces_time);
  Py_XDECREF(args);
  Py_XDECREF(kwargs);
  Py_XDECREF(sec);
  Py_XDECREF(nanosec);
  Py_XDECREF(time_obj);
  Py_XDECREF(pheader);
  Py_XDECREF(pframe_id);
  Py_XDECREF(ptransform);
  Py_XDECREF(ptranslation);
  Py_XDECREF(protation);
  Py_XDECREF(child_frame_id);
  Py_XDECREF(ptrans_x);
  Py_XDECREF(ptrans_y);
  Py_XDECREF(ptrans_z);
  Py_XDECREF(prot_x);
  Py_XDECREF(prot_y);
  Py_XDECREF(prot_z);
  Py_XDECREF(prot_w);
  return pinst;
}

static builtin_interfaces::msg::Time toMsg(const tf2::TimePoint & t)
{
  std::chrono::nanoseconds ns = t.time_since_epoch();
  std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(ns);
  ns -= s;
  builtin_interfaces::msg::Time time_msg;
  time_msg.sec = (int32_t)s.count();
  time_msg.nanosec = (uint32_t)ns.count();
  return time_msg;
}

static tf2::TimePoint fromMsg(const builtin_interfaces::msg::Time & time_msg)
{
  std::chrono::nanoseconds ns = std::chrono::seconds(time_msg.sec) + std::chrono::nanoseconds(
    time_msg.nanosec);
  return tf2::TimePoint(ns);
}

static int getOptionalTimeAttribute(PyObject * obj, const char * name, PyObject ** value)
{
#if PY_VERSION_HEX >= 0x030D0000
  return PyObject_GetOptionalAttrString(obj, name, value);
#else
  *value = PyObject_GetAttrString(obj, name);
  if (*value) {
    return 1;
  }
  if (PyErr_ExceptionMatches(PyExc_AttributeError)) {
    PyErr_Clear();
    return 0;
  }
  return -1;
#endif
}

static int timeParts(PyObject * obj, PyObject ** seconds, PyObject ** nanoseconds)
{
  const int has_seconds = getOptionalTimeAttribute(obj, "sec", seconds);
  if (has_seconds <= 0) {
    return has_seconds;
  }
  const int has_nanoseconds = getOptionalTimeAttribute(obj, "nanosec", nanoseconds);
  if (has_nanoseconds <= 0) {
    Py_CLEAR(*seconds);
  }
  return has_nanoseconds;
}

static int timeInNanoseconds(PyObject * obj, int64_t * value, const char * kind)
{
  PyObject * nanoseconds;
  const int found = getOptionalTimeAttribute(obj, "nanoseconds", &nanoseconds);
  if (found == 0) {
    PyErr_Format(PyExc_TypeError, "%s must have sec and nanosec, or nanoseconds.", kind);
  }
  if (found <= 0) {
    return 0;
  }

  *value = PyLong_AsLongLong(nanoseconds);
  Py_DECREF(nanoseconds);
  return PyErr_Occurred() ? 0 : 1;
}

static int rostime_converter(PyObject * obj, tf2::TimePoint * rt)
{
  PyObject * sec;
  PyObject * nanosec;
  const int found = timeParts(obj, &sec, &nanosec);
  if (found < 0) {
    return 0;
  }
  if (found == 1) {
    builtin_interfaces::msg::Time msg;
    msg.sec = PyLong_AsLong(sec);
    msg.nanosec = PyLong_AsUnsignedLong(nanosec);
    Py_DECREF(nanosec);
    Py_DECREF(sec);
    if (PyErr_Occurred()) {
      return 0;
    }
    *rt = fromMsg(msg);
    return 1;
  }

  int64_t nanoseconds;
  if (!timeInNanoseconds(obj, &nanoseconds, "time")) {
    return 0;
  }
  *rt = tf2::TimePoint(std::chrono::nanoseconds(nanoseconds));
  return 1;
}

static int rosduration_converter(PyObject * obj, tf2::Duration * rt)
{
  PyObject * sec;
  PyObject * nanosec;
  const int found = timeParts(obj, &sec, &nanosec);
  if (found < 0) {
    return 0;
  }
  if (found == 1) {
    const auto seconds = PyLong_AsLong(sec);
    const auto nanoseconds = PyLong_AsUnsignedLong(nanosec);
    Py_DECREF(nanosec);
    Py_DECREF(sec);
    if (PyErr_Occurred()) {
      return 0;
    }
    *rt = std::chrono::seconds(seconds) + std::chrono::nanoseconds(nanoseconds);
    return 1;
  }

  int64_t nanoseconds;
  if (!timeInNanoseconds(obj, &nanoseconds, "duration")) {
    return 0;
  }
  *rt = std::chrono::duration_cast<tf2::Duration>(std::chrono::nanoseconds(nanoseconds));
  return 1;
}

static PyObject * BufferCore_new(PyTypeObject * type, PyObject * args, PyObject * kw)
{
  PyObject * self = PyType_GenericNew(type, args, kw);
  if (self) {
    new (&reinterpret_cast<buffer_core_t *>(self)->bc) std::shared_ptr<tf2::BufferCore>();
  }
  return self;
}

static int BufferCore_init(PyObject * self, PyObject * args, PyObject * kw)
{
  (void)kw;

  tf2::Duration cache_time(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME);

  if (!PyArg_ParseTuple(args, "|O&", rosduration_converter, &cache_time)) {
    return -1;
  }

  try {
    std::atomic_store(
      &reinterpret_cast<buffer_core_t *>(self)->bc, std::make_shared<tf2::BufferCore>(cache_time));
  } catch (const std::bad_alloc &) {
    PyErr_NoMemory();
    return -1;
  }

  return 0;
}

static void BufferCore_dealloc(PyObject * self)
{
  auto * type = Py_TYPE(self);
  PyObject_GC_UnTrack(self);
  std::destroy_at(&reinterpret_cast<buffer_core_t *>(self)->bc);
  type->tp_free(self);
  Py_DECREF(type);
}

static int BufferCore_traverse(PyObject * self, visitproc visit, void * arg)
{
  Py_VISIT(Py_TYPE(self));
  return 0;
}

static PyObject * allFramesAsYAML(PyObject * self, PyObject * args)
{
  (void)args;
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  return stringToPython(bc->allFramesAsYAML());
}

static PyObject * allFramesAsString(PyObject * self, PyObject * args)
{
  (void)args;
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  return stringToPython(bc->allFramesAsString());
}

static PyObject * canTransformCore(PyObject * self, PyObject * args, PyObject * kw)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * target_frame, * source_frame;
  tf2::TimePoint time;
  static const char * keywords[] = {"target_frame", "source_frame", "time", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "ssO&",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)), &target_frame,
      &source_frame, rostime_converter, &time))
  {
    return nullptr;
  }
  std::string error_msg;
  bool can_transform = bc->canTransform(target_frame, source_frame, time, &error_msg);
  // return PyBool_FromLong(t->canTransform(target_frame, source_frame, time));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

static PyObject * canTransformFullCore(PyObject * self, PyObject * args, PyObject * kw)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * target_frame, * source_frame, * fixed_frame;
  tf2::TimePoint target_time, source_time;
  static const char * keywords[] =
  {"target_frame", "target_time", "source_frame", "source_time", "fixed_frame", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "sO&sO&s",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)),
      &target_frame,
      rostime_converter,
      &target_time,
      &source_frame,
      rostime_converter,
      &source_time,
      &fixed_frame))
  {
    return nullptr;
  }
  std::string error_msg;
  bool can_transform = bc->canTransform(
    target_frame, target_time, source_frame, source_time,
    fixed_frame, &error_msg);
  // return PyBool_FromLong(t->canTransform(target_frame, target_time, source_frame,
  //                                        source_time, fixed_frame));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

static PyObject * asListOfStrings(std::vector<std::string> los)
{
  PyObject * r = PyList_New(los.size());
  size_t i;
  for (i = 0; i < los.size(); i++) {
    PyList_SetItem(r, i, stringToPython(los[i]));
  }
  return r;
}

static PyObject * _chain(PyObject * self, PyObject * args, PyObject * kw)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * target_frame, * source_frame, * fixed_frame;
  tf2::TimePoint target_time, source_time;
  std::vector<std::string> output;
  static const char * keywords[] =
  {"target_frame", "target_time", "source_frame", "source_time", "fixed_frame", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "sO&sO&s",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)),
      &target_frame,
      rostime_converter,
      &target_time,
      &source_frame,
      rostime_converter,
      &source_time,
      &fixed_frame))
  {
    return nullptr;
  }

  WRAP(
    bc->_chainAsVector(
      target_frame, target_time, source_frame, source_time, fixed_frame,
      output));
  return asListOfStrings(output);
}

static PyObject * getLatestCommonTime(PyObject * self, PyObject * args)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * target_frame, * source_frame;
  tf2::CompactFrameID target_id, source_id;
  tf2::TimePoint tf2_time;
  std::string error_string;

  if (!PyArg_ParseTuple(args, "ss", &target_frame, &source_frame)) {
    return nullptr;
  }
  WRAP(target_id = bc->_validateFrameId("get_latest_common_time", target_frame));
  WRAP(source_id = bc->_validateFrameId("get_latest_common_time", source_frame));
  const tf2::TF2Error r = bc->_getLatestCommonTime(target_id, source_id, tf2_time, &error_string);

  if (r != tf2::TF2Error::TF2_NO_ERROR) {
    const auto * state = moduleState(self);
    if (state) {
      PyErr_SetString(state->transform_exception, error_string.c_str());
    }
    return nullptr;
  }

  builtin_interfaces::msg::Time time_msg;
  PyObject * rclpy_time = nullptr;
  PyObject * call_args = nullptr;
  PyObject * kwargs = nullptr;
  PyObject * seconds = nullptr;
  PyObject * nanoseconds = nullptr;
  PyObject * ob = nullptr;

  rclpy_time = importAttribute("rclpy.time", "Time");
  if (!rclpy_time) {
    goto cleanup;
  }
  time_msg = toMsg(tf2_time);
  call_args = PyTuple_New(0);
  if (!call_args) {
    goto cleanup;
  }
  kwargs = PyDict_New();
  if (!kwargs) {
    goto cleanup;
  }
  seconds = Py_BuildValue("i", time_msg.sec);
  if (!seconds) {
    goto cleanup;
  }
  nanoseconds = Py_BuildValue("i", time_msg.nanosec);
  if (!nanoseconds) {
    goto cleanup;
  }
  if (0 != PyDict_SetItemString(kwargs, "seconds", seconds)) {
    goto cleanup;
  }
  if (0 != PyDict_SetItemString(kwargs, "nanoseconds", nanoseconds)) {
    goto cleanup;
  }

  ob = PyObject_Call(rclpy_time, call_args, kwargs);

cleanup:
  if (PyErr_Occurred()) {
    Py_XDECREF(ob);
    ob = nullptr;
  }
  Py_XDECREF(rclpy_time);
  Py_XDECREF(call_args);
  Py_XDECREF(kwargs);
  Py_XDECREF(seconds);
  Py_XDECREF(nanoseconds);
  return ob;
}

static PyObject * lookupTransformCore(PyObject * self, PyObject * args, PyObject * kw)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * target_frame, * source_frame;
  tf2::TimePoint time;
  static const char * keywords[] = {"target_frame", "source_frame", "time", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "ssO&",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)), &target_frame,
      &source_frame, rostime_converter, &time))
  {
    return nullptr;
  }
  geometry_msgs::msg::TransformStamped transform;
  WRAP(transform = bc->lookupTransform(target_frame, source_frame, time));
  // TODO(anyone): Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
  // return Py_BuildValue("(ddd)(dddd)",
  //    origin.x, origin.y, origin.z,
  //    rotation.x, rotation.y, rotation.z, rotation.w);
}

static PyObject * lookupTransformFullCore(PyObject * self, PyObject * args, PyObject * kw)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * target_frame, * source_frame, * fixed_frame;
  tf2::TimePoint target_time, source_time;
  static const char * keywords[] =
  {"target_frame", "target_time", "source_frame", "source_time", "fixed_frame", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "sO&sO&s",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)),
      &target_frame,
      rostime_converter,
      &target_time,
      &source_frame,
      rostime_converter,
      &source_time,
      &fixed_frame))
  {
    return nullptr;
  }
  geometry_msgs::msg::TransformStamped transform;
  WRAP(
    transform =
    bc->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
  // TODO(anyone): Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
}

static PyObject * lookupVelocityCore(PyObject * self, PyObject * args, PyObject * kw)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * tracking_frame, * observation_frame;
  tf2::TimePoint time;
  tf2::Duration averaging_interval;
  static const char * keywords[] =
  {"tracking_frame", "observation_frame", "time", "averaging_interval", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "ssO&O&", (char **)keywords, &tracking_frame,
      &observation_frame, rostime_converter, &time, rosduration_converter, &averaging_interval))
  {
    return nullptr;
  }
  geometry_msgs::msg::VelocityStamped velocity_stamped;
  WRAP(
    velocity_stamped =
    bc->lookupVelocity(tracking_frame, observation_frame, time, averaging_interval));

  return Py_BuildValue(
    "(ddd)(ddd)",
    velocity_stamped.velocity.linear.x, velocity_stamped.velocity.linear.y,
    velocity_stamped.velocity.linear.z,
    velocity_stamped.velocity.angular.x, velocity_stamped.velocity.angular.y,
    velocity_stamped.velocity.angular.z);
}

static PyObject * lookupVelocityFullCore(PyObject * self, PyObject * args)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * tracking_frame, * observation_frame, * reference_frame, * reference_point_frame;
  tf2::TimePoint time;
  tf2::Duration averaging_interval;
  double px, py, pz;

  if (!PyArg_ParseTuple(
      args, "sss(ddd)sO&O&",
      &tracking_frame,
      &observation_frame,
      &reference_frame,
      &px, &py, &pz,
      &reference_point_frame,
      rostime_converter, &time,
      rosduration_converter, &averaging_interval))
  {
    return nullptr;
  }
  geometry_msgs::msg::VelocityStamped velocity_stamped;
  tf2::Vector3 pt(px, py, pz);
  WRAP(
    velocity_stamped =
    bc->lookupVelocity(
      tracking_frame, observation_frame, reference_frame, pt,
      reference_point_frame, time, averaging_interval));

  return Py_BuildValue(
    "(ddd)(ddd)",
    velocity_stamped.velocity.linear.x, velocity_stamped.velocity.linear.y,
    velocity_stamped.velocity.linear.z,
    velocity_stamped.velocity.angular.x, velocity_stamped.velocity.angular.y,
    velocity_stamped.velocity.angular.z);
}

static inline int checkTranslationType(PyObject * o)
{
  PyTypeObject * translation_type =
    reinterpret_cast<PyTypeObject *>(importAttribute(
      "geometry_msgs.msg",
      "Vector3"));
  if (!translation_type) {
    return 0;
  }
  int type_check = PyObject_TypeCheck(o, translation_type);
  Py_DECREF(reinterpret_cast<PyObject *>(translation_type));
  int attr_check = PyObject_HasAttrString(o, "x") &&
    PyObject_HasAttrString(o, "y") &&
    PyObject_HasAttrString(o, "z");
  if (!type_check) {
    PyErr_WarnEx(PyExc_UserWarning, "translation should be of type Vector3", 1);
    return type_check;
  }
  return attr_check;
}

static inline int checkRotationType(PyObject * o)
{
  PyTypeObject * rotation_type =
    reinterpret_cast<PyTypeObject *>(importAttribute(
      "geometry_msgs.msg",
      "Quaternion"));
  if (!rotation_type) {
    return 0;
  }
  int type_check = PyObject_TypeCheck(o, rotation_type);
  Py_DECREF(reinterpret_cast<PyObject *>(rotation_type));
  int attr_check = PyObject_HasAttrString(o, "w") &&
    PyObject_HasAttrString(o, "x") &&
    PyObject_HasAttrString(o, "y") &&
    PyObject_HasAttrString(o, "z");
  if (!type_check) {
    PyErr_WarnEx(PyExc_UserWarning, "translation should be of type Quaternion", 1);
  }
  return attr_check;
}

static PyObject * setTransform(PyObject * self, PyObject * args)
{
  PyObject * ret = nullptr;
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  PyObject * py_transform;
  char * authority;
  tf2::TimePoint time;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority)) {
    return nullptr;
  }

  geometry_msgs::msg::TransformStamped transform;

  PyObject * header = nullptr;
  PyObject * stamp = nullptr;
  PyObject * frame_id = nullptr;
  PyObject * child_frame_id = nullptr;

  PyObject * mtransform = nullptr;

  PyObject * translation = nullptr;
  PyObject * tx = nullptr;
  PyObject * ty = nullptr;
  PyObject * tz = nullptr;

  PyObject * rotation = nullptr;
  PyObject * rx = nullptr;
  PyObject * ry = nullptr;
  PyObject * rz = nullptr;
  PyObject * rw = nullptr;

  header = PyObject_GetAttrString(py_transform, "header");
  if (!header) {
    goto cleanup;
  }
  stamp = PyObject_GetAttrString(header, "stamp");
  if (!stamp) {
    goto cleanup;
  }
  frame_id = PyObject_GetAttrString(header, "frame_id");
  if (!frame_id) {
    goto cleanup;
  }
  child_frame_id = PyObject_GetAttrString(py_transform, "child_frame_id");
  if (!child_frame_id) {
    goto cleanup;
  }

  mtransform = PyObject_GetAttrString(py_transform, "transform");
  if (!mtransform) {
    goto cleanup;
  }

  translation = PyObject_GetAttrString(mtransform, "translation");
  if (!translation) {
    goto cleanup;
  }
  tx = PyObject_GetAttrString(translation, "x");
  if (!tx) {
    goto cleanup;
  }
  ty = PyObject_GetAttrString(translation, "y");
  if (!ty) {
    goto cleanup;
  }
  tz = PyObject_GetAttrString(translation, "z");
  if (!tz) {
    goto cleanup;
  }

  rotation = PyObject_GetAttrString(mtransform, "rotation");
  if (!rotation) {
    goto cleanup;
  }
  rx = PyObject_GetAttrString(rotation, "x");
  if (!rx) {
    goto cleanup;
  }
  ry = PyObject_GetAttrString(rotation, "y");
  if (!ry) {
    goto cleanup;
  }
  rz = PyObject_GetAttrString(rotation, "z");
  if (!rz) {
    goto cleanup;
  }
  rw = PyObject_GetAttrString(rotation, "w");
  if (!rw) {
    goto cleanup;
  }

  transform.header.frame_id = stringFromPython(frame_id);

  if (rostime_converter(stamp, &time) != 1) {
    goto cleanup;
  }

  transform.child_frame_id = stringFromPython(child_frame_id);
  if (PyErr_Occurred()) {
    goto cleanup;
  }

  transform.header.stamp = toMsg(time);

  if (!checkTranslationType(translation)) {
    PyErr_SetString(PyExc_TypeError, "transform.translation must have members x, y, z");
    goto cleanup;
  }

  transform.transform.translation.x = PyFloat_AsDouble(tx);
  transform.transform.translation.y = PyFloat_AsDouble(ty);
  transform.transform.translation.z = PyFloat_AsDouble(tz);

  if (!checkRotationType(rotation)) {
    PyErr_SetString(PyExc_TypeError, "transform.rotation must have members w, x, y, z");
    goto cleanup;
  }

  transform.transform.rotation.x = PyFloat_AsDouble(rx);
  transform.transform.rotation.y = PyFloat_AsDouble(ry);
  transform.transform.rotation.z = PyFloat_AsDouble(rz);
  transform.transform.rotation.w = PyFloat_AsDouble(rw);

  bc->setTransform(transform, authority);

  Py_INCREF(Py_None);
  ret = Py_None;

cleanup:
  Py_XDECREF(rw);
  Py_XDECREF(rz);
  Py_XDECREF(ry);
  Py_XDECREF(rx);
  Py_XDECREF(rotation);
  Py_XDECREF(tz);
  Py_XDECREF(ty);
  Py_XDECREF(tx);
  Py_XDECREF(translation);
  Py_XDECREF(mtransform);
  Py_XDECREF(child_frame_id);
  Py_XDECREF(frame_id);
  Py_XDECREF(stamp);
  Py_XDECREF(header);

  return ret;
}

static PyObject * setTransformStatic(PyObject * self, PyObject * args)
{
  PyObject * ret = nullptr;
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  PyObject * py_transform;
  char * authority;
  tf2::TimePoint time;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority)) {
    return nullptr;
  }

  geometry_msgs::msg::TransformStamped transform;

  PyObject * header = nullptr;
  PyObject * stamp = nullptr;
  PyObject * child_frame_id = nullptr;
  PyObject * frame_id = nullptr;
  PyObject * mtransform = nullptr;
  PyObject * translation = nullptr;
  PyObject * tx = nullptr;
  PyObject * ty = nullptr;
  PyObject * tz = nullptr;
  PyObject * rotation = nullptr;
  PyObject * rx = nullptr;
  PyObject * ry = nullptr;
  PyObject * rz = nullptr;
  PyObject * rw = nullptr;

  header = PyObject_GetAttrString(py_transform, "header");
  if (!header) {
    goto cleanup;
  }
  stamp = PyObject_GetAttrString(header, "stamp");
  if (!stamp) {
    goto cleanup;
  }
  child_frame_id = PyObject_GetAttrString(py_transform, "child_frame_id");
  if (!child_frame_id) {
    goto cleanup;
  }
  frame_id = PyObject_GetAttrString(header, "frame_id");
  if (!frame_id) {
    goto cleanup;
  }
  mtransform = PyObject_GetAttrString(py_transform, "transform");
  if (!mtransform) {
    goto cleanup;
  }
  translation = PyObject_GetAttrString(mtransform, "translation");
  if (!translation) {
    goto cleanup;
  }
  tx = PyObject_GetAttrString(translation, "x");
  if (!tx) {
    goto cleanup;
  }
  ty = PyObject_GetAttrString(translation, "y");
  if (!ty) {
    goto cleanup;
  }
  tz = PyObject_GetAttrString(translation, "z");
  if (!tz) {
    goto cleanup;
  }
  rotation = PyObject_GetAttrString(mtransform, "rotation");
  if (!rotation) {
    goto cleanup;
  }
  rx = PyObject_GetAttrString(rotation, "x");
  if (!rx) {
    goto cleanup;
  }
  ry = PyObject_GetAttrString(rotation, "y");
  if (!ry) {
    goto cleanup;
  }
  rz = PyObject_GetAttrString(rotation, "z");
  if (!rz) {
    goto cleanup;
  }
  rw = PyObject_GetAttrString(rotation, "w");
  if (!rw) {
    goto cleanup;
  }

  transform.child_frame_id = stringFromPython(child_frame_id);
  transform.header.frame_id = stringFromPython(frame_id);
  if (PyErr_Occurred()) {
    goto cleanup;
  }

  if (rostime_converter(stamp, &time) != 1) {
    goto cleanup;
  }
  transform.header.stamp = toMsg(time);

  if (!checkTranslationType(translation)) {
    PyErr_SetString(PyExc_TypeError, "transform.translation must be of type Vector3");
    goto cleanup;
  }

  transform.transform.translation.x = PyFloat_AsDouble(tx);
  transform.transform.translation.y = PyFloat_AsDouble(ty);
  transform.transform.translation.z = PyFloat_AsDouble(tz);

  if (!checkRotationType(rotation)) {
    PyErr_SetString(PyExc_TypeError, "transform.rotation must be of type Quaternion");
    goto cleanup;
  }

  transform.transform.rotation.x = PyFloat_AsDouble(rx);
  transform.transform.rotation.y = PyFloat_AsDouble(ry);
  transform.transform.rotation.z = PyFloat_AsDouble(rz);
  transform.transform.rotation.w = PyFloat_AsDouble(rw);

  // only difference to above is is_static == True
  bc->setTransform(transform, authority, true);

  Py_INCREF(Py_None);
  ret = Py_None;

cleanup:
  Py_XDECREF(rw);
  Py_XDECREF(rz);
  Py_XDECREF(ry);
  Py_XDECREF(rx);
  Py_XDECREF(rotation);
  Py_XDECREF(tz);
  Py_XDECREF(ty);
  Py_XDECREF(tx);
  Py_XDECREF(translation);
  Py_XDECREF(mtransform);
  Py_XDECREF(frame_id);
  Py_XDECREF(child_frame_id);
  Py_XDECREF(stamp);
  Py_XDECREF(header);

  return ret;
}

static PyObject * clear(PyObject * self, PyObject * args)
{
  (void)args;
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  bc->clear();
  Py_RETURN_NONE;
}

static PyObject * _frameExists(PyObject * self, PyObject * args)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  char * frame_id_str;
  if (!PyArg_ParseTuple(args, "s", &frame_id_str)) {
    return nullptr;
  }
  return PyBool_FromLong(bc->_frameExists(frame_id_str));
}

static PyObject * _getFrameStrings(PyObject * self, PyObject * args)
{
  (void)args;
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  std::vector<std::string> ids;
  bc->_getFrameStrings(ids);
  return asListOfStrings(ids);
}

static PyObject * _allFramesAsDot(PyObject * self, PyObject * args, PyObject * kw)
{
  const auto bc = getBufferCore(self);
  if (!bc) {
    return nullptr;
  }
  static const char * keywords[] = {"time", nullptr};
  tf2::TimePoint time;
  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "|O&",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)), rostime_converter, &time))
  {
    return nullptr;
  }
  return stringToPython(bc->_allFramesAsDot(time).c_str());
}


static struct PyMethodDef buffer_core_methods[] =
{
  {"all_frames_as_yaml", allFramesAsYAML, METH_VARARGS, nullptr},
  {"all_frames_as_string", allFramesAsString, METH_VARARGS, nullptr},
  {"set_transform", setTransform, METH_VARARGS, nullptr},
  {"set_transform_static", setTransformStatic, METH_VARARGS, nullptr},
  {"can_transform_core", (PyCFunction)canTransformCore, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"can_transform_full_core", (PyCFunction)canTransformFullCore, METH_VARARGS | METH_KEYWORDS,
    nullptr},
  {"_chain", (PyCFunction)_chain, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"clear", (PyCFunction)clear, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"_frameExists", (PyCFunction)_frameExists, METH_VARARGS, nullptr},
  {"_getFrameStrings", (PyCFunction)_getFrameStrings, METH_VARARGS, nullptr},
  {"_allFramesAsDot", (PyCFunction)_allFramesAsDot, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"get_latest_common_time", (PyCFunction)getLatestCommonTime, METH_VARARGS, nullptr},
  {"lookup_transform_core", (PyCFunction)lookupTransformCore, METH_VARARGS | METH_KEYWORDS,
    nullptr},
  {"lookup_transform_full_core", (PyCFunction)lookupTransformFullCore, METH_VARARGS | METH_KEYWORDS,
    nullptr},
  {"lookupVelocityCore", (PyCFunction)lookupVelocityCore, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"lookupVelocityFullCore", lookupVelocityFullCore, METH_VARARGS, nullptr},
  {nullptr, nullptr, 0, nullptr}
};

static PyType_Slot buffer_core_slots[] = {
  {Py_tp_methods, buffer_core_methods},
  {Py_tp_init, reinterpret_cast<void *>(BufferCore_init)},
  {Py_tp_dealloc, reinterpret_cast<void *>(BufferCore_dealloc)},
  {Py_tp_new, reinterpret_cast<void *>(BufferCore_new)},
  {Py_tp_traverse, reinterpret_cast<void *>(BufferCore_traverse)},
  {0, nullptr},
};

static PyType_Spec buffer_core_spec = {
  "_tf2.BufferCore",
  sizeof(buffer_core_t),
  0,
  Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_HAVE_GC | Py_TPFLAGS_IMMUTABLETYPE,
  buffer_core_slots,
};

struct ExceptionSpec
{
  const char * name;
  PyObject * TfModuleState::* member;
};

static constexpr ExceptionSpec exceptions[] = {
  {"TransformException", &TfModuleState::transform_exception},
  {"ConnectivityException", &TfModuleState::connectivity_exception},
  {"LookupException", &TfModuleState::lookup_exception},
  {"ExtrapolationException", &TfModuleState::extrapolation_exception},
  {"InvalidArgumentException", &TfModuleState::invalid_argument_exception},
  {"TimeoutException", &TfModuleState::timeout_exception},
};

static int moduleExec(PyObject * module)
{
  auto * state = static_cast<TfModuleState *>(PyModule_GetState(module));
  for (const auto & spec : exceptions) {
    const std::string qualified_name = std::string("tf2.") + spec.name;
    auto & exception = state->*(spec.member);
    exception = PyErr_NewException(qualified_name.c_str(), state->transform_exception, nullptr);
    if (!exception || PyModule_AddObjectRef(module, spec.name, exception) < 0) {
      return -1;
    }
  }

  PyObject * type = PyType_FromModuleAndSpec(module, &buffer_core_spec, nullptr);
  if (!type) {
    return -1;
  }
  const int result = PyModule_AddObjectRef(module, "BufferCore", type);
  Py_DECREF(type);
  return result;
}

static int moduleTraverse(PyObject * module, visitproc visit, void * arg)
{
  const auto * state = static_cast<TfModuleState *>(PyModule_GetState(module));
  if (state) {
    for (const auto & spec : exceptions) {
      Py_VISIT(state->*(spec.member));
    }
  }
  return 0;
}

static int moduleClear(PyObject * module)
{
  auto * state = static_cast<TfModuleState *>(PyModule_GetState(module));
  if (state) {
    for (const auto & spec : exceptions) {
      Py_CLEAR(state->*(spec.member));
    }
  }
  return 0;
}

static PyModuleDef_Slot tf_module_slots[] = {
  {Py_mod_exec, reinterpret_cast<void *>(moduleExec)},
#if PY_VERSION_HEX >= 0x030C0000
  {Py_mod_multiple_interpreters, Py_MOD_MULTIPLE_INTERPRETERS_NOT_SUPPORTED},
#endif
#ifdef Py_GIL_DISABLED
  {Py_mod_gil, Py_MOD_GIL_NOT_USED},
#endif
  {0, nullptr},
};

PyModuleDef tf_module = {
  PyModuleDef_HEAD_INIT,
  "_tf2_py",
  nullptr,
  sizeof(TfModuleState),
  nullptr,
  tf_module_slots,
  moduleTraverse,
  moduleClear,
  nullptr,
};

PyMODINIT_FUNC PyInit__tf2_py()
{
  return PyModuleDef_Init(&tf_module);
}
