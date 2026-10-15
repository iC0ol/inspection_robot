
# Copyright 2026 inspection_robot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Provide an isolated ROS domain for the pytest session."""

import os

import domain_coordinator


_domain_scope = None
_previous_domain = None


def pytest_configure(config):
    """Reserve a ROS domain before running package tests."""
    global _domain_scope, _previous_domain

    _previous_domain = os.environ.get('ROS_DOMAIN_ID')

    _domain_scope = domain_coordinator.domain_id()
    selected_domain = _domain_scope.__enter__()

    os.environ['ROS_DOMAIN_ID'] = str(selected_domain)

    print(
        f'\nMission tests ROS_DOMAIN_ID={selected_domain}'
    )


def pytest_unconfigure(config):
    """Restore the environment and release the ROS domain."""
    global _domain_scope

    if _domain_scope is None:
        return

    if _previous_domain is None:
        os.environ.pop('ROS_DOMAIN_ID', None)
    else:
        os.environ['ROS_DOMAIN_ID'] = _previous_domain

    _domain_scope.__exit__(None, None, None)
    _domain_scope = None
