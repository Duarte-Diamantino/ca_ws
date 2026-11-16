"""Length scale handling for the planner GUI."""

import math


class ScaleMixin:
    """Provides unit conversion helpers and scale update logic."""

    def ppm(self):
        # [ca_ws] escala fixa vinda de um map.yaml (load_map_yaml): 1/resolution px/m. Sem isto a
        # escala era grid_spacing * N, i.e. a imagem media sempre 24/5 = 4.8 m de altura (a pista
        # FNR tem 7.0 m) e mexer no slider da grelha mudava a escala do mundo.
        if getattr(self, "fixed_ppm", None):  # [ca_ws]
            return float(self.fixed_ppm)  # [ca_ws]
        if not self.grid_spacing:
            return 1.0
        if self.scale_mode.get() == "sqm":
            return float(self.grid_spacing) * float(self.squares_per_meter)
        mps = float(self.meters_per_square)
        if mps <= 0:
            mps = 1e-6
        return float(self.grid_spacing) / mps

    def m2px(self, meters):
        return float(meters) * self.ppm()

    def px2m(self, pixels):
        ppm = self.ppm()
        return float(pixels) / ppm if ppm else float(pixels)

    def _set_sqm(self, value):
        try:
            self.squares_per_meter = float(value)
        except Exception:
            return
        if self.scale_mode.get() == "sqm":
            self.update_scale()

    def _set_mps(self, value):
        try:
            self.meters_per_square = float(value)
        except Exception:
            return
        if self.scale_mode.get() == "mps":
            self.update_scale()

    def update_scale(self):
        if self.scale_mode.get() == "sqm":
            try:
                self.squares_per_meter = float(self.sqm_var.get())
            except Exception:
                pass
        else:
            try:
                self.meters_per_square = float(self.mps_var.get())
            except Exception:
                pass

        ppm = self.ppm()
        px_per_sq = float(self.grid_spacing) if self.grid_spacing else 0.0
        if self.scale_mode.get() == "sqm":
            msg = (
                f"1 m = {self.squares_per_meter:.2f} quad  |  "
                f"{ppm:.1f} px/m  |  1 quad = {self.px2m(px_per_sq):.3f} m"
            )
        else:
            msg = f"1 quad = {self.meters_per_square:.3f} m  |  {ppm:.1f} px/m"
        if getattr(self, "fixed_ppm", None):  # [ca_ws]
            msg = f"map.yaml: {ppm:.1f} px/m ({1.0 / ppm:.5f} m/px)  |  1 quad = {self.px2m(px_per_sq):.3f} m"  # [ca_ws]

        try:
            self.lbl_scale_info.config(text=msg)
        except Exception:
            pass

        self.min_turning_radius = self.wheelbase / math.tan(self.max_steering_angle)
        self.lbl_rt.config(text=f"Raio mínimo: {self.min_turning_radius:.2f} m")
        self.redraw()

    def update_max_speed(self, value):
        self.max_speed = float(value)

    def update_max_steering(self, value):
        self.max_steering_angle = math.radians(float(value))
        self.min_turning_radius = self.wheelbase / math.tan(self.max_steering_angle)
        self.lbl_rt.config(text=f"Raio mínimo: {self.min_turning_radius:.2f} m")
