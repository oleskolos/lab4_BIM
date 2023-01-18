import StdReinfShapeBuilder.GeneralReinfShapeBuilder as GeneralShapeBuilder
import StdReinfShapeBuilder.LinearBarPlacementBuilder as LinearBarBuilder

from StdReinfShapeBuilder.ConcreteCoverProperties import ConcreteCoverProperties
from StdReinfShapeBuilder.ReinforcementShapeProperties import ReinforcementShapeProperties
from StdReinfShapeBuilder.RotationAngles import RotationAngles
import NemAll_Python_Reinforcement as AllplanReinf

import NemAll_Python_Geometry as geo
import NemAll_Python_BaseElements as base
import NemAll_Python_BasisElements as basis
import NemAll_Python_Utility as util
import val as val
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties

print('--- Loading of Reif ---')


class BeamReif:

    def __init__(self, doc):
        self.modelList = []
        self.handleList = []
        self.document = doc

    def create(self, buildE):
        self.top(buildE)
        self.create_handles(buildE)

        return (self.modelList, self.handleList)

    def get(self, buildE):
        self.width_down = buildE.width_down.value
        self.len = buildE.len.value
        self.height_bottom = buildE.height_bottom.value
        self.cut_botT = buildE.cut_botT.value
        self.cut_botB = buildE.cut_botB.value
        self.centerWidth = buildE.centerWidth.value
        self.middleHeight = buildE.middleHeight.value
        self.Rad = buildE.Rad.value
        self.widthTop = buildE.widthTop.value
        self.heightTop = buildE.heightTop.value
        self.PlSpace = buildE.PlSpace.value
        self.PlHeight = buildE.PlHeight.value
        self.color = buildE.color.value
        self.cut_topT = buildE.cut_topT.value
        self.Deep = buildE.Deep.value
        self.BarSpacing = buildE.BarSpacing.value

    def reif_create(self, buildE):
        self.get(buildE)
        modelAng = RotationAngles(0, 90, 90)
        steelGr = AllplanReinf.ReinforcementSettings.GetSteelGrade()
        shape_props = ReinforcementShapeProperties.rebar(
            25, 4, steelGr, -1, AllplanReinf.BendingShapeType.LongitudinalBar)
        coverProps = ConcreteCoverProperties.left_right_bottom(
            20. * 2, 20. * 2, 20.)
        shape = GeneralShapeBuilder.create_longitudinal_shape_with_hooks(
            self.Deep, modelAng, shape_props, coverProps, 0, -1)
        height = self.height_bottom + self.middleHeight + \
            self.heightTop + self.PlHeight + 200
        self.modelList.append(LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(1, shape, geo.Point3D(
            self.widthTop / 6, 0, height), geo.Point3D(self.widthTop / 6, self.len, height), 0, 0, self.BarSpacing))
        modelAng = RotationAngles(0, 90, 270)
        steelGr = AllplanReinf.ReinforcementSettings.GetSteelGrade()
        shape_props = ReinforcementShapeProperties.rebar(
            25, 4, steelGr, -1, AllplanReinf.BendingShapeType.LongitudinalBar)
        coverProps = ConcreteCoverProperties.left_right_bottom(
            20. * 2, 20. * 2, 20.)
        shape = GeneralShapeBuilder.create_longitudinal_shape_with_hooks(
            self.Deep, modelAng, shape_props, coverProps, 0, -1)
        self.modelList.append(LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(1, shape, geo.Point3D(
            self.widthTop - self.widthTop / 3, 0, height), geo.Point3D(self.widthTop - self.widthTop / 3, self.len, height), 0, 0, self.BarSpacing))

    def bottom(self, buildE):
        self.get(buildE)
        f = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(0, 0, 0),
                                       geo.Vector3D(1, 0, 0),
                                       geo.Vector3D(0, 0, 1)),
            self.width_down,
            self.len,
            self.height_bottom)

        f_i = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(0, 0, 0),
                                       geo.Vector3D(1, 0, 0),
                                       geo.Vector3D(0, 0, 1)),
            self.width_down,
            self.len,
            self.height_bottom)

        c_w = self.cut_botT
        c_w_b = self.cut_botB

        if c_w > 0:
            ed = util.VecSizeTList()
            ed.append(1)
            ed.append(3)

            e, f = geo.ChamferCalculus.Calculate(
                f, ed, c_w, False)

            if not val.polyhedron(e):
                return

        if c_w_b > 0:
            ed2 = util.VecSizeTList()
            ed2.append(8)
            ed2.append(10)

            e, f_i = geo.ChamferCalculus.Calculate(
                f_i, ed2, c_w_b, False)

            if not val.polyhedron(e):
                return

        e, end = geo.MakeIntersection(f, f_i)

        return end

    def middle(self, buildE):
        self.get(buildE)
        f = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(self.width_down / 2 - self.centerWidth / 2, 0, self.height_bottom),
                                       geo.Vector3D(1, 0, 0),
                                       geo.Vector3D(0, 0, 1)),
            self.centerWidth,
            self.len,
            self.middleHeight)

        ce = geo.BRep3D.CreateCylinder(
            geo.AxisPlacement3D(geo.Point3D(self.cut_botT, self.len / 8, self.height_bottom + self.middleHeight / 2),
                                       geo.Vector3D(0, 0, 1),
                                       geo.Vector3D(1, 0, 0)),
            self.Rad, self.centerWidth)

        ce1 = geo.BRep3D.CreateCylinder(
            geo.AxisPlacement3D(geo.Point3D(self.cut_botT, self.len - self.len / 8, self.height_bottom + self.middleHeight / 2),
                                       geo.Vector3D(0, 0, 1),
                                       geo.Vector3D(1, 0, 0)),
            self.Rad, self.centerWidth)

        e, f = geo.MakeSubtraction(f, ce)
        e, f = geo.MakeSubtraction(f, ce1)

        e, end = geo.MakeUnion(
            f, self.bottom(buildE))
        return end

    def top(self, buildE):
        self.get(buildE)
        f = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(0 - (self.widthTop - self.width_down) / 2, 0, self.height_bottom + self.middleHeight),
                                       geo.Vector3D(1, 0, 0),
                                       geo.Vector3D(0, 0, 1)),
            self.widthTop,
            self.len,
            self.heightTop)

        f_p = geo.BRep3D.CreateCuboid(
            geo.AxisPlacement3D(geo.Point3D(self.PlSpace - (self.widthTop - self.width_down) / 2, 0, self.height_bottom + self.middleHeight + self.heightTop),
                                       geo.Vector3D(1, 0, 0),
                                       geo.Vector3D(0, 0, 1)),
            self.widthTop - self.PlSpace*2,
            self.len,
            self.PlHeight)

        com_prop = base.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = self.color

        chamfer_widthTopop = self.cut_topT

        if chamfer_widthTopop > 0:
            ed2 = util.VecSizeTList()
            ed2.append(8)
            ed2.append(10)

            e, f = geo.ChamferCalculus.Calculate(
                f, ed2, chamfer_widthTopop, False)

            if not val.polyhedron(e):
                return

        e, end = geo.MakeUnion(
            f, self.middle(buildE))
        e, end = geo.MakeUnion(end, f_p)
        self.modelList.append(
            basis.ModelElement3D(com_prop, end))
        self.reif_create(buildE)

    def create_handles(self, buildE):
        self.get(buildE)
        origin = geo.Point3D(
            self.width_down / 2, self.len, self.middleHeight + self.height_bottom)
        origin2 = geo.Point3D(
            self.width_down / 2, 0, self.height_bottom / 2)
        origin3 = geo.Point3D(
            0, self.len, (self.height_bottom - self.cut_botT) / 2)
        origin4 = geo.Point3D(
            0 - (self.widthTop - self.width_down) / 2, self.len, self.middleHeight + self.height_bottom + self.cut_topT)
        origin5 = geo.Point3D(
            self.width_down / 2, self.len, self.middleHeight + self.height_bottom - self.height_bottom / 4)
        origin6 = geo.Point3D(
            self.width_down / 2, self.len, self.middleHeight + self.height_bottom + self.heightTop)
        origin7 = geo.Point3D(
            self.width_down / 2, self.len, 0)
        origin8 = geo.Point3D(
            self.width_down / 2 - self.centerWidth / 2, self.len, self.middleHeight / 2 + self.height_bottom)

        self.handleList.append(
            HandleProperties("middleHeight",
                             geo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z),
                             geo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z - self.middleHeight),
                             [("middleHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handleList.append(
            HandleProperties("len",
                             geo.Point3D(origin2.X,
                                                origin2.Y + self.len,
                                                origin2.Z),
                             geo.Point3D(origin2.X,
                                                origin2.Y,
                                                origin2.Z),
                             [("len", HandleDirection.y_dir)],
                             HandleDirection.y_dir,
                             False))

        self.handleList.append(
            HandleProperties("width_down", geo.Point3D(origin3.X + self.width_down, origin3.Y, origin3.Z),
                             geo.Point3D(
                                 origin3.X, origin3.Y, origin3.Z),
                             [("width_down", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handleList.append(
            HandleProperties("widthTop",
                             geo.Point3D(origin4.X + self.widthTop,
                                                origin4.Y,
                                                origin4.Z),
                             geo.Point3D(origin4.X,
                                                origin4.Y,
                                                origin4.Z),
                             [("widthTop", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handleList.append(
            HandleProperties("heightTop",
                             geo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z + self.heightTop),
                             geo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z),
                             [("heightTop", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handleList.append(
            HandleProperties("PlHeight",
                             geo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z + self.PlHeight),
                             geo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z),
                             [("PlHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handleList.append(
            HandleProperties("height_bottom",
                             geo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z + self.height_bottom),
                             geo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z),
                             [("height_bottom", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handleList.append(
            HandleProperties("centerWidth",
                             geo.Point3D(origin8.X + self.centerWidth,
                                                origin8.Y,
                                                origin8.Z),
                             geo.Point3D(origin8.X,
                                                origin8.Y,
                                                origin8.Z),
                             [("centerWidth", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))


def check_allplan_version(buildE, version):
    del buildE
    del version
    return True


def create_element(buildE, doc):
    element = BeamReif(doc)
    return element.create(buildE)


def move_handle(buildE, handle_prop, input_pnt, doc):
    buildE.change_property(handle_prop, input_pnt)
    return create_element(buildE, doc)
