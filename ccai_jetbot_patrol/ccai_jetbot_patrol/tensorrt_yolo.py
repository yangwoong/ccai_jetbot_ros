"""Minimal TensorRT engine runner for a YOLO detection head.

This targets TensorRT 7.x (Jetson Nano / JetPack 4.6.x, L4T r32.7.1), which
uses the binding-index API (engine.get_binding_shape / execute_async_v2)
rather than the newer named-tensor API from TensorRT 8.5+/10. It follows the
standard buffer-allocation pattern from NVIDIA's own TensorRT samples
(samples/python/common.py), since that pattern is stable and well-tested
across years of Jetson deployments.

Engines are hardware/TensorRT-version specific and are NOT portable (unlike
the .onnx file) - build one on the Jetson itself with
scripts/build_yolo_tensorrt_engine.sh, never commit it to git.

Import this module lazily and wrap all use in try/except: tensorrt/pycuda
may not be installed, and any failure here should fall back to the existing
OpenCV DNN ONNX path.
"""

import numpy as np
import pycuda.autoinit  # noqa: F401  (creates/attaches the CUDA context)
import pycuda.driver as cuda
import tensorrt as trt


class HostDeviceMem:
    def __init__(self, host_mem, device_mem) -> None:
        self.host = host_mem
        self.device = device_mem


class TensorRTYolo:
    def __init__(self, engine_path: str) -> None:
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError("failed to deserialize TensorRT engine: {0}".format(engine_path))

        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.output_shape = None

        for index in range(self.engine.num_bindings):
            shape = self.engine.get_binding_shape(index)
            size = abs(int(trt.volume(shape)))
            dtype = trt.nptype(self.engine.get_binding_dtype(index))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings.append(int(device_mem))
            if self.engine.binding_is_input(index):
                self.inputs.append(HostDeviceMem(host_mem, device_mem))
            else:
                self.outputs.append(HostDeviceMem(host_mem, device_mem))
                self.output_shape = tuple(shape)

        if not self.inputs or not self.outputs:
            raise RuntimeError("engine has no input/output bindings: {0}".format(engine_path))

    def infer(self, blob: "np.ndarray") -> "np.ndarray":
        np.copyto(self.inputs[0].host, blob.ravel())
        cuda.memcpy_htod_async(self.inputs[0].device, self.inputs[0].host, self.stream)
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out.host, out.device, self.stream)
        self.stream.synchronize()
        output = self.outputs[0].host
        if self.output_shape is not None:
            output = output.reshape(self.output_shape)
        return output
