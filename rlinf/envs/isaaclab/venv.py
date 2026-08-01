# Copyright 2025 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from multiprocessing.connection import Connection

import torch
import torch.multiprocessing as mp

from .utils import CloudpickleWrapper


def _torch_worker(
    child_remote: Connection,
    parent_remote: Connection,
    env_fn_wrapper: CloudpickleWrapper,
    action_queue: mp.Queue,
    obs_queue: mp.Queue,
    reset_idx_queue: mp.Queue,
):
    parent_remote.close()
    isaac_env = None
    sim_app = None
    try:
        env_fn = env_fn_wrapper.x
        isaac_env, sim_app = env_fn()
        device = isaac_env.device
        while True:
            try:
                cmd = child_remote.recv()
            except EOFError:
                child_remote.close()
                break
            if cmd == "reset":
                reset_index, reset_seed = reset_idx_queue.get()
                if reset_index is None:
                    reset_result = isaac_env.reset(seed=reset_seed)
                else:
                    reset_result = isaac_env.reset(
                        seed=reset_seed, env_ids=reset_index.to(device)
                    )
                obs_queue.put(reset_result)
            elif cmd == "step":
                input_action = action_queue.get()
                step_result = isaac_env.step(input_action)
                obs_queue.put(step_result)
            elif cmd == "close":
                isaac_env.close()
                child_remote.close()
                sim_app.close()
                break
            elif cmd == "device":
                child_remote.send(isaac_env.device)
            else:
                child_remote.close()
                raise NotImplementedError
    except (KeyboardInterrupt, Exception) as e:
        child_remote.close()
        print(f"[IsaacLabEnvWorker] Terminated: {e}", flush=True)
    finally:
        if isaac_env is not None:
            try:
                isaac_env.close()
            except Exception as e:
                print(f"[IsaacLabEnvWorker] Env close error: {e}", flush=True)
        if sim_app is not None:
            try:
                sim_app.close()
            except Exception as e:
                print(f"[IsaacLabEnvWorker] SimApp close error: {e}", flush=True)


class SubProcIsaacLabEnv:
    def __init__(self, env_fn):
        mp.set_start_method("spawn", force=True)
        ctx = mp.get_context("spawn")
        self.parent_remote, self.child_remote = ctx.Pipe(duplex=True)
        self.action_queue = ctx.Queue()
        self.obs_queue = ctx.Queue()
        self.reset_idx = ctx.Queue()
        args = (
            self.child_remote,
            self.parent_remote,
            CloudpickleWrapper(env_fn),
            self.action_queue,
            self.obs_queue,
            self.reset_idx,
        )
        self.isaac_lab_process = ctx.Process(
            target=_torch_worker, args=args, daemon=True
        )
        self.isaac_lab_process.start()
        self.child_remote.close()

    def _check_child_alive(self):
        """Check if the child process is still alive; raise if it died unexpectedly."""
        if not self.isaac_lab_process.is_alive():
            exitcode = self.isaac_lab_process.exitcode
            raise RuntimeError(
                f"IsaacLab Env Worker (pid={self.isaac_lab_process.pid}) "
                f"died unexpectedly with exit code {exitcode}. "
                f"Check the logs for errors during IsaacSim initialization."
            )

    def reset(self, seed=None, env_ids=None):
        self._check_child_alive()
        self.parent_remote.send("reset")
        self.reset_idx.put((env_ids, seed))
        try:
            obs, info = self.obs_queue.get(timeout=300)
        except Exception:
            self._check_child_alive()
            raise RuntimeError(
                "Timeout waiting for IsaacLab Env Worker to complete reset(). "
                "The child process may have crashed during IsaacSim initialization."
            )
        return obs, info

    def step(self, action: torch.Tensor):
        """
        action : (bs, action_dim)
        """
        self._check_child_alive()
        self.parent_remote.send("step")
        self.action_queue.put(action)
        try:
            env_step_result = self.obs_queue.get(timeout=300)
        except Exception:
            self._check_child_alive()
            raise RuntimeError(
                "Timeout waiting for IsaacLab Env Worker to complete step(). "
                "The child process may have crashed during IsaacSim execution."
            )
        return env_step_result

    def close(self):
        if not self.isaac_lab_process.is_alive():
            return
        try:
            self.parent_remote.send("close")
        except (BrokenPipeError, EOFError, OSError):
            pass
        self.isaac_lab_process.join(timeout=10)
        self.isaac_lab_process.terminate()

    def device(self):
        self.parent_remote.send("device")
        return self.parent_remote.recv()
