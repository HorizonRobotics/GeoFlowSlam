from DatasetReader import DatasetReader
from PoseTransformer import PoseTransformer
from PoseEvaluator import PoseEvaluator
from VehicleKinematic import VehicleKinematic
import importlib


class LazyImport:
    def __init__(self, module_name, class_name):
        self.module_name = module_name
        self.class_name = class_name
        self._class = None

    def __call__(self, *args, **kwargs):
        if self._class is None:
            module = importlib.import_module(self.module_name)
            self._class = getattr(module, self.class_name)
        return self._class(*args, **kwargs)


ImagePackReader = LazyImport('horizon_driving_dataset.RawPackReader',
                             'ImagePackReader')
RawPackReader = LazyImport('horizon_driving_dataset.RawPackReader',
                           'RawPackReader')
DatasetPackReader = LazyImport('horizon_driving_dataset.DatasetPackReader',
                               'DatasetPackReader')
