"""
Autofocus module for multi-object detection and focusing.
Módulo de autofoco multi-objeto.
"""

from .multi_object_autofocus import (
    MultiObjectAutofocusController,
    DetectedObject,
    FocusedCapture
)
from .smart_focus_scorer import SmartFocusScorer
from .focus_metric import calculate_focus_score, build_multifocal_z_positions
from .roi_tracker import RoiTracker
from .bpof_candidates import (
    BpofCandidateTable,
    FocusCandidate,
    find_isolated_dips,
    min_candidates_for_planes,
    symmetric_fine_window,
)
from .af_kpi import AfCycleKpi, AfSessionKpi
from .fine_scan_plan import RingDeclineStop, center_out_sequence, ring_counts
from .persisted_params import sanitize_autofocus_form
from .stack_plan import rebalance_symmetric, stack_asymmetry_ratio
from .z_prior import BpofPrior, bootstrap_window

__all__ = [
    'MultiObjectAutofocusController',
    'DetectedObject',
    'FocusedCapture',
    'SmartFocusScorer',
    'calculate_focus_score',
    'build_multifocal_z_positions',
    'RoiTracker',
    'BpofCandidateTable',
    'FocusCandidate',
    'find_isolated_dips',
    'min_candidates_for_planes',
    'symmetric_fine_window',
    'AfCycleKpi',
    'AfSessionKpi',
    'RingDeclineStop',
    'center_out_sequence',
    'ring_counts',
    'sanitize_autofocus_form',
    'rebalance_symmetric',
    'stack_asymmetry_ratio',
    'BpofPrior',
    'bootstrap_window',
]
