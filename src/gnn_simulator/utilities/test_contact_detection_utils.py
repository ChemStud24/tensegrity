"""
Unit tests for contact detection utilities.
Tests all signed distance functions and the get_dist_fn dispatcher with various scenarios including:
- Separated objects
- Touching objects
- Penetrating objects
- Edge cases (parallel, coincident, etc.)
- Batched inputs
- Rectangular plane interactions
- Function dispatch for all supported object type combinations
"""

import unittest
import torch
from gnn_simulator.utilities.contact_detection_utils import (
    sphere_sphere_signed_distance,
    sphere_cylinder_signed_distance,
    cylinder_cylinder_signed_distance,
    sphere_static_prism_signed_distance,
    cylinder_static_prism_signed_distance,
    sphere_static_rect_plane_signed_distance,
    get_dist_fn
)
from gnn_simulator.state_objects.primitive_shapes import SphereState, Cylinder, StaticPrism, StaticRectPlane
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE


class TestSphereSphereSignedDistance(unittest.TestCase):
    """Test cases for sphere-sphere signed distance."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.device = torch.device('cpu')
        self.dtype = DEFAULT_DTYPE
    
    def create_sphere(self, center, radius, name="sphere"):
        """Helper to create a sphere."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1
        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)
        
        if isinstance(radius, (int, float)):
            radius_tensor = torch.tensor([[radius]], dtype=self.dtype)
        elif len(radius.shape) == 0:
            radius_tensor = radius.unsqueeze(0).unsqueeze(0)
        elif len(radius.shape) == 1:
            radius_tensor = radius.unsqueeze(-1)
        else:
            radius_tensor = radius
        
        # solid_sphere_body expects scalar mass, so use a Python float
        mass_scalar = 1.0  # Python float, not tensor
        sphere = SphereState(
            name=name,
            center=center,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius_tensor,
            mass=mass_scalar,  # Pass Python scalar directly
            principal_axis=torch.tensor([[0, 0, 1]], dtype=self.dtype).repeat(batch_size, 1, 1),
            sites={}
        )
        # Manually set the mass to the correct batched value after creation
        sphere.mass = torch.ones(batch_size, 1, 1, dtype=self.dtype)
        return sphere
    
    def test_separated_spheres(self):
        """Test two separated spheres."""
        sphere1 = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        sphere2 = self.create_sphere(torch.tensor([[5.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        
        p1, p2, dist = sphere_sphere_signed_distance(sphere1, sphere2)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be positive (separated)
        self.assertGreater(dist.item(), 0)
        # Expected distance: 5 - 1 - 1 = 3
        self.assertAlmostEqual(dist.item(), 3.0, places=5)
        
        # Points should be on surfaces
        dist_p1 = torch.linalg.norm(p1 - sphere1.pos, dim=1).item()
        dist_p2 = torch.linalg.norm(p2 - sphere2.pos, dim=1).item()
        self.assertAlmostEqual(dist_p1, 1.0, places=5)
        self.assertAlmostEqual(dist_p2, 1.0, places=5)
    
    def test_touching_spheres(self):
        """Test two touching spheres."""
        sphere1 = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        sphere2 = self.create_sphere(torch.tensor([[2.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        
        p1, p2, dist = sphere_sphere_signed_distance(sphere1, sphere2)
        
        # Distance should be approximately zero (touching)
        self.assertAlmostEqual(dist.item(), 0.0, places=4)
    
    def test_penetrating_spheres(self):
        """Test two penetrating spheres."""
        sphere1 = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        sphere2 = self.create_sphere(torch.tensor([[1.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        
        p1, p2, dist = sphere_sphere_signed_distance(sphere1, sphere2)
        
        # Distance should be negative (penetrating)
        self.assertLess(dist.item(), 0)
        # Expected distance: 1 - 1 - 1 = -1
        self.assertAlmostEqual(dist.item(), -1.0, places=5)
    
    def test_batched_spheres(self):
        """Test batched sphere inputs."""
        batch_size = 3
        centers1 = torch.tensor([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ], dtype=self.dtype)
        centers2 = torch.tensor([
            [5.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
        ], dtype=self.dtype)
        # Use scalar radius - solid_sphere_body doesn't support batched radii
        # The radius will be broadcast to all batches
        radius1 = 1.0
        radius2 = 1.0
        
        sphere1 = self.create_sphere(centers1, radius1)
        sphere2 = self.create_sphere(centers2, radius2)
        
        p1, p2, dist = sphere_sphere_signed_distance(sphere1, sphere2)
        
        # Check output shapes
        self.assertEqual(p1.shape, (batch_size, 3, 1))
        self.assertEqual(p2.shape, (batch_size, 3, 1))
        self.assertEqual(dist.shape, (batch_size, 1))
        
        # Check expected distances
        expected_dists = torch.tensor([3.0, 0.0, -1.0], dtype=self.dtype)
        torch.testing.assert_close(dist.squeeze(-1), expected_dists, rtol=1e-5, atol=1e-5)


class TestSphereCylinderSignedDistance(unittest.TestCase):
    """Test cases for sphere-cylinder signed distance."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.device = torch.device('cpu')
        self.dtype = DEFAULT_DTYPE
    
    def create_sphere(self, center, radius, name="sphere"):
        """Helper to create a sphere."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1
        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)
        
        if isinstance(radius, (int, float)):
            radius = torch.tensor([[radius]], dtype=self.dtype)
        elif len(radius.shape) == 0:
            radius = radius.unsqueeze(0).unsqueeze(0)
        elif len(radius.shape) == 1:
            radius = radius.unsqueeze(-1)
        
        return SphereState(
            name=name,
            center=center,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius,
            mass=torch.ones(batch_size, 1, 1, dtype=self.dtype),
            principal_axis=torch.tensor([[0, 0, 1]], dtype=self.dtype).repeat(batch_size, 1, 1),
            sites={}
        )
    
    def create_cylinder(self, end_pt1, end_pt2, radius, name="cylinder"):
        """Helper to create a cylinder."""
        batch_size = end_pt1.shape[0] if len(end_pt1.shape) > 1 else 1
        
        if len(end_pt1.shape) == 1:
            end_pt1 = end_pt1.unsqueeze(0).unsqueeze(-1)
            end_pt2 = end_pt2.unsqueeze(0).unsqueeze(-1)
        elif len(end_pt1.shape) == 2:
            end_pt1 = end_pt1.unsqueeze(-1)
            end_pt2 = end_pt2.unsqueeze(-1)
        
        # Reshape to (batch_size, 3, 2) format
        end_pts = torch.cat([end_pt1, end_pt2], dim=-1)  # (batch_size, 3, 2)
        
        if isinstance(radius, (int, float)):
            radius = torch.tensor([[radius]], dtype=self.dtype)
        elif len(radius.shape) == 0:
            radius = radius.unsqueeze(0).unsqueeze(0)
        elif len(radius.shape) == 1:
            radius = radius.unsqueeze(-1)
        
        return Cylinder(
            name=name,
            end_pts=end_pts,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius,
            mass=torch.ones(batch_size, 1, 1, dtype=self.dtype),
            sites={}
        )
    
    def test_sphere_away_from_cylinder(self):
        """Test sphere separated from cylinder."""
        sphere = self.create_sphere(torch.tensor([[5.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        cylinder = self.create_cylinder(
            torch.tensor([[0.0, 0.0, -1.0]], dtype=self.dtype),
            torch.tensor([[0.0, 0.0, 1.0]], dtype=self.dtype),
            0.5
        )
        
        p1, p2, dist = sphere_cylinder_signed_distance(sphere, cylinder)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be positive
        self.assertGreater(dist.item(), 0)
    
    def test_sphere_on_cylinder_axis(self):
        """Test sphere centered on cylinder axis."""
        sphere = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 0.3)
        cylinder = self.create_cylinder(
            torch.tensor([[0.0, 0.0, -1.0]], dtype=self.dtype),
            torch.tensor([[0.0, 0.0, 1.0]], dtype=self.dtype),
            0.5
        )
        
        p1, p2, dist = sphere_cylinder_signed_distance(sphere, cylinder)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be negative (penetrating)
        self.assertLess(dist.item(), 0)
    
    def test_sphere_perpendicular_to_cylinder(self):
        """Test sphere positioned perpendicular to cylinder axis."""
        sphere = self.create_sphere(torch.tensor([[2.0, 0.0, 0.0]], dtype=self.dtype), 0.5)
        cylinder = self.create_cylinder(
            torch.tensor([[0.0, 0.0, -1.0]], dtype=self.dtype),
            torch.tensor([[0.0, 0.0, 1.0]], dtype=self.dtype),
            0.5
        )
        
        p1, p2, dist = sphere_cylinder_signed_distance(sphere, cylinder)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be positive (separated)
        self.assertGreater(dist.item(), 0)
        # Expected: distance from sphere center to cylinder surface - sphere radius
        # = (2.0 - 0.5) - 0.5 = 1.0
        self.assertAlmostEqual(dist.item(), 1.0, places=4)


class TestCylinderCylinderSignedDistance(unittest.TestCase):
    """Test cases for cylinder-cylinder signed distance."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.device = torch.device('cpu')
        self.dtype = DEFAULT_DTYPE
    
    def create_cylinder(self, end_pt1, end_pt2, radius, name="cylinder"):
        """Helper to create a cylinder."""
        batch_size = end_pt1.shape[0] if len(end_pt1.shape) > 1 else 1
        
        if len(end_pt1.shape) == 1:
            end_pt1 = end_pt1.unsqueeze(0).unsqueeze(-1)
            end_pt2 = end_pt2.unsqueeze(0).unsqueeze(-1)
        elif len(end_pt1.shape) == 2:
            end_pt1 = end_pt1.unsqueeze(-1)
            end_pt2 = end_pt2.unsqueeze(-1)
        
        # Reshape to (batch_size, 3, 2) format
        end_pts = torch.cat([end_pt1, end_pt2], dim=-1)  # (batch_size, 3, 2)
        
        if isinstance(radius, (int, float)):
            radius = torch.tensor([[[radius]]], dtype=self.dtype)
        elif len(radius.shape) < 3:
            radius = radius.reshape(1, 1, 1)

        return Cylinder(
            name=name,
            end_pts=end_pts,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius,
            mass=torch.ones(1, 1, 1, dtype=self.dtype),
            sites={}
        )
    
    def test_parallel_cylinders(self):
        """Test two parallel cylinders."""
        cylinder1 = self.create_cylinder(
            torch.tensor([[0.0, 0.0, -1.0]], dtype=self.dtype),
            torch.tensor([[0.0, 0.0, 1.0]], dtype=self.dtype),
            0.5
        )
        cylinder2 = self.create_cylinder(
            torch.tensor([[2.0, 0.0, -1.0]], dtype=self.dtype),
            torch.tensor([[2.0, 0.0, 1.0]], dtype=self.dtype),
            0.5
        )
        
        p1, p2, dist = cylinder_cylinder_signed_distance(cylinder1, cylinder2)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be positive (separated)
        self.assertGreater(dist.item(), 0)
        # Expected: 2.0 - 0.5 - 0.5 = 1.0
        self.assertAlmostEqual(dist.item(), 1.0, places=4)
    
    def test_perpendicular_cylinders(self):
        """Test two perpendicular cylinders."""
        cylinder1 = self.create_cylinder(
            torch.tensor([[-1.0, 0.0, 0.0]], dtype=self.dtype),
            torch.tensor([[1.0, 0.0, 0.0]], dtype=self.dtype),
            0.5
        )
        cylinder2 = self.create_cylinder(
            torch.tensor([[0.0, -1.0, 0.0]], dtype=self.dtype),
            torch.tensor([[0.0, 1.0, 0.0]], dtype=self.dtype),
            0.5
        )
        
        p1, p2, dist = cylinder_cylinder_signed_distance(cylinder1, cylinder2)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be negative (penetrating at origin)
        self.assertLess(dist.item(), 0)


class TestSphereStaticPrismSignedDistance(unittest.TestCase):
    """Test cases for sphere-static prism signed distance."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.device = torch.device('cpu')
        self.dtype = DEFAULT_DTYPE
    
    def create_sphere(self, center, radius, name="sphere"):
        """Helper to create a sphere."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1
        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)
        
        if isinstance(radius, (int, float)):
            radius = torch.tensor([[radius]], dtype=self.dtype)
        elif len(radius.shape) == 0:
            radius = radius.unsqueeze(0).unsqueeze(0)
        elif len(radius.shape) == 1:
            radius = radius.unsqueeze(-1)
        
        return SphereState(
            name=name,
            center=center,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius,
            mass=torch.ones(batch_size, 1, 1, dtype=self.dtype),
            principal_axis=torch.tensor([[0, 0, 1]], dtype=self.dtype).repeat(batch_size, 1, 1),
            sites={}
        )
    
    def create_prism(self, center, rot_mat, half_lens, name="prism"):
        """Helper to create a static prism."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1
        
        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)
        
        if len(rot_mat.shape) == 2:
            rot_mat = rot_mat.unsqueeze(0)
        if rot_mat.shape[0] == 1 and batch_size > 1:
            rot_mat = rot_mat.repeat(batch_size, 1, 1)
        
        if isinstance(half_lens, (list, tuple)):
            half_lens = tuple(torch.tensor([[hl]], dtype=self.dtype) for hl in half_lens)
        elif isinstance(half_lens, torch.Tensor):
            if len(half_lens.shape) == 1:
                half_lens = tuple(half_lens[i].unsqueeze(0).unsqueeze(0) for i in range(3))
        
        return StaticPrism(
            name=name,
            center=center,
            rot_mat=rot_mat,
            half_lens=half_lens,
            dtype=self.dtype
        )
    
    def test_sphere_outside_prism(self):
        """Test sphere outside prism."""
        sphere = self.create_sphere(torch.tensor([[5.0, 0.0, 0.0]], dtype=self.dtype), 0.5)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        prism = self.create_prism(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype))
        )
        
        p1, p2, dist = sphere_static_prism_signed_distance(sphere, prism)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be positive (separated)
        self.assertGreater(dist.item(), 0)
    
    def test_sphere_inside_prism(self):
        """Test sphere inside prism."""
        sphere = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 0.3)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        prism = self.create_prism(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype))
        )
        
        p1, p2, dist = sphere_static_prism_signed_distance(sphere, prism)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be negative (penetrating)
        self.assertLess(dist.item(), 0)
    
    def test_sphere_on_prism_surface(self):
        """Test sphere touching prism surface."""
        sphere = self.create_sphere(torch.tensor([[1.5, 0.0, 0.0]], dtype=self.dtype), 0.5)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        prism = self.create_prism(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype))
        )
        
        p1, p2, dist = sphere_static_prism_signed_distance(sphere, prism)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be approximately zero (touching)
        self.assertAlmostEqual(dist.item(), 0.0, places=3)


class TestCylinderStaticPrismSignedDistance(unittest.TestCase):
    """Test cases for cylinder-static prism signed distance."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.device = torch.device('cpu')
        self.dtype = DEFAULT_DTYPE
    
    def create_cylinder(self, end_pt1, end_pt2, radius, name="cylinder"):
        """Helper to create a cylinder."""
        batch_size = end_pt1.shape[0] if len(end_pt1.shape) > 1 else 1
        
        if len(end_pt1.shape) == 1:
            end_pt1 = end_pt1.unsqueeze(0).unsqueeze(-1)
            end_pt2 = end_pt2.unsqueeze(0).unsqueeze(-1)
        elif len(end_pt1.shape) == 2:
            end_pt1 = end_pt1.unsqueeze(-1)
            end_pt2 = end_pt2.unsqueeze(-1)
        
        # Reshape to (batch_size, 3, 2) format
        end_pts = torch.cat([end_pt1, end_pt2], dim=-1)  # (batch_size, 3, 2)
        
        if isinstance(radius, (int, float)):
            radius = torch.tensor([[radius]], dtype=self.dtype)
        elif len(radius.shape) == 0:
            radius = radius.unsqueeze(0).unsqueeze(0)
        elif len(radius.shape) == 1:
            radius = radius.unsqueeze(-1)
        
        return Cylinder(
            name=name,
            end_pts=end_pts,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius,
            mass=torch.ones(batch_size, 1, 1, dtype=self.dtype),
            sites={}
        )
    
    def create_prism(self, center, rot_mat, half_lens, name="prism"):
        """Helper to create a static prism."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1
        
        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)
        
        if len(rot_mat.shape) == 2:
            rot_mat = rot_mat.unsqueeze(0)
        if rot_mat.shape[0] == 1 and batch_size > 1:
            rot_mat = rot_mat.repeat(batch_size, 1, 1)
        
        if isinstance(half_lens, (list, tuple)):
            half_lens = tuple(torch.tensor([[hl]], dtype=self.dtype) for hl in half_lens)
        elif isinstance(half_lens, torch.Tensor):
            if len(half_lens.shape) == 1:
                half_lens = tuple(half_lens[i].unsqueeze(0).unsqueeze(0) for i in range(3))
        
        return StaticPrism(
            name=name,
            center=center,
            rot_mat=rot_mat,
            half_lens=half_lens,
            dtype=self.dtype
        )
    
    def test_cylinder_parallel_to_prism_face(self):
        """Test cylinder parallel to prism face."""
        cylinder = self.create_cylinder(
            torch.tensor([[0.0, 0.0, -1.0]], dtype=self.dtype),
            torch.tensor([[0.0, 0.0, 1.0]], dtype=self.dtype),
            0.3
        )
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        prism = self.create_prism(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype))
        )
        
        p1, p2, dist = cylinder_static_prism_signed_distance(cylinder, prism)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be negative (penetrating)
        self.assertLess(dist.item(), 0)
    
    def test_cylinder_perpendicular_to_prism(self):
        """Test cylinder perpendicular to prism."""
        cylinder = self.create_cylinder(
            torch.tensor([[3.0, 0.0, 0.0]], dtype=self.dtype),
            torch.tensor([[5.0, 0.0, 0.0]], dtype=self.dtype),
            0.3
        )
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        prism = self.create_prism(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype))
        )
        
        p1, p2, dist = cylinder_static_prism_signed_distance(cylinder, prism)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be positive (separated)
        self.assertGreater(dist.item(), 0)
    
    def test_cylinder_prism_with_custom_sampling(self):
        """Test cylinder-prism with custom sampling parameters."""
        cylinder = self.create_cylinder(
            torch.tensor([[3.0, 0.0, 0.0]], dtype=self.dtype),
            torch.tensor([[5.0, 0.0, 0.0]], dtype=self.dtype),
            0.3
        )
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        prism = self.create_prism(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype))
        )
        
        # Test with custom sampling parameters
        p1, p2, dist = cylinder_static_prism_signed_distance(cylinder, prism, num_axis_samples=10, num_circ_samples=12)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be positive (separated)
        self.assertGreater(dist.item(), 0)
        
        # Test with default parameters (should also work)
        p1_default, p2_default, dist_default = cylinder_static_prism_signed_distance(cylinder, prism)
        
        # Check output shapes
        self.assertEqual(p1_default.shape, (1, 3, 1))
        self.assertEqual(p2_default.shape, (1, 3, 1))
        self.assertEqual(dist_default.shape, (1, 1))
        
        # Both should give positive distances (may differ slightly due to sampling)
        self.assertGreater(dist_default.item(), 0)


class TestSphereStaticRectPlaneSignedDistance(unittest.TestCase):
    """Test cases for sphere-static rectangular plane signed distance."""

    def setUp(self):
        """Set up test fixtures."""
        self.device = torch.device('cpu')
        self.dtype = DEFAULT_DTYPE

    def create_sphere(self, center, radius, name="sphere"):
        """Helper to create a sphere."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1
        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)

        if isinstance(radius, (int, float)):
            radius = torch.tensor([[radius]], dtype=self.dtype)
        elif len(radius.shape) == 0:
            radius = radius.unsqueeze(0).unsqueeze(0)
        elif len(radius.shape) == 1:
            radius = radius.unsqueeze(-1)

        return SphereState(
            name=name,
            center=center,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius,
            mass=torch.ones(batch_size, 1, 1, dtype=self.dtype),
            principal_axis=torch.tensor([[0, 0, 1]], dtype=self.dtype).repeat(batch_size, 1, 1),
            sites={}
        )

    def create_rect_plane(self, center, rot_mat, half_lens, name="plane"):
        """Helper to create a static rectangular plane."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1

        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)

        if len(rot_mat.shape) == 2:
            rot_mat = rot_mat.unsqueeze(0)
        if rot_mat.shape[0] == 1 and batch_size > 1:
            rot_mat = rot_mat.repeat(batch_size, 1, 1)

        if isinstance(half_lens, (list, tuple)) and not isinstance(half_lens[0], torch.Tensor):
            half_lens = tuple(torch.tensor([[hl]], dtype=self.dtype) for hl in half_lens)

        return StaticRectPlane(
            name=name,
            center=center,
            rot_mat=rot_mat,
            half_lens=half_lens,
            dtype=self.dtype
        )

    def test_sphere_above_plane(self):
        """Test sphere above a horizontal plane (positive z side)."""
        sphere = self.create_sphere(torch.tensor([[0.0, 0.0, 3.0]], dtype=self.dtype), 0.5)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        plane = self.create_rect_plane(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[2.0]], dtype=self.dtype),
             torch.tensor([[2.0]], dtype=self.dtype))
        )

        p1, p2, dist = sphere_static_rect_plane_signed_distance(sphere, plane)

        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))

        # Distance should be positive (separated, on +z side)
        self.assertGreater(dist.item(), 0)
        # Expected: 3.0 - 0.5 = 2.5
        self.assertAlmostEqual(dist.item(), 2.5, places=4)

    def test_sphere_below_plane(self):
        """Test sphere below a horizontal plane (negative z side)."""
        sphere = self.create_sphere(torch.tensor([[0.0, 0.0, -3.0]], dtype=self.dtype), 0.5)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        plane = self.create_rect_plane(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[2.0]], dtype=self.dtype),
             torch.tensor([[2.0]], dtype=self.dtype))
        )

        p1, p2, dist = sphere_static_rect_plane_signed_distance(sphere, plane)

        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))

        # Distance should be negative (on -z side: sign=-1, so -3.0 - 0.5 = -3.5)
        self.assertLess(dist.item(), 0)

    def test_sphere_touching_plane(self):
        """Test sphere touching the plane surface."""
        sphere = self.create_sphere(torch.tensor([[0.0, 0.0, 0.5]], dtype=self.dtype), 0.5)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        plane = self.create_rect_plane(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[2.0]], dtype=self.dtype),
             torch.tensor([[2.0]], dtype=self.dtype))
        )

        p1, p2, dist = sphere_static_rect_plane_signed_distance(sphere, plane)

        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))

        # Distance should be approximately zero (touching)
        self.assertAlmostEqual(dist.item(), 0.0, places=4)

    def test_sphere_off_edge_of_plane(self):
        """Test sphere positioned outside the plane's rectangular bounds."""
        sphere = self.create_sphere(torch.tensor([[5.0, 0.0, 1.0]], dtype=self.dtype), 0.5)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        plane = self.create_rect_plane(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[1.0]], dtype=self.dtype),
             torch.tensor([[1.0]], dtype=self.dtype))
        )

        p1, p2, dist = sphere_static_rect_plane_signed_distance(sphere, plane)

        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))

        # Sphere is far from the finite plane - distance should be positive
        self.assertGreater(dist.item(), 0)

    def test_sphere_centered_on_plane(self):
        """Test sphere whose center is exactly on the plane surface."""
        sphere = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 0.5)
        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        plane = self.create_rect_plane(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[2.0]], dtype=self.dtype),
             torch.tensor([[2.0]], dtype=self.dtype))
        )

        p1, p2, dist = sphere_static_rect_plane_signed_distance(sphere, plane)

        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))

        # Sphere center is on the plane: signed_distance = sign(0)*0 - 0.5 = -0.5
        self.assertAlmostEqual(dist.item(), -0.5, places=4)

    def test_rotated_plane(self):
        """Test sphere above a rotated plane (plane normal along x-axis)."""
        sphere = self.create_sphere(torch.tensor([[3.0, 0.0, 0.0]], dtype=self.dtype), 0.5)
        # Rotation: z-axis -> x-axis (rotate 90 deg around y)
        rot_mat = torch.tensor([
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0]
        ], dtype=self.dtype).unsqueeze(0)
        plane = self.create_rect_plane(
            torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype),
            rot_mat,
            (torch.tensor([[2.0]], dtype=self.dtype),
             torch.tensor([[2.0]], dtype=self.dtype))
        )

        p1, p2, dist = sphere_static_rect_plane_signed_distance(sphere, plane)

        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))

        # Sphere is on the positive normal side of the rotated plane
        # Distance should be positive: 3.0 - 0.5 = 2.5
        self.assertGreater(dist.item(), 0)
        self.assertAlmostEqual(dist.item(), 2.5, places=4)


class TestGetDistFn(unittest.TestCase):
    """Test cases for the get_dist_fn dispatch function."""

    def setUp(self):
        """Set up test fixtures."""
        self.dtype = DEFAULT_DTYPE

        # Create one of each object type for testing dispatch
        center = torch.tensor([[[0.0], [0.0], [0.0]]], dtype=self.dtype)
        self.sphere = SphereState(
            name="sphere",
            center=center,
            linear_vel=torch.zeros(1, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(1, 3, 1, dtype=self.dtype),
            radius=torch.tensor([[1.0]], dtype=self.dtype),
            mass=torch.ones(1, 1, 1, dtype=self.dtype),
            principal_axis=torch.tensor([[[0], [0], [1]]], dtype=self.dtype),
            sites={}
        )

        end_pts = torch.tensor([[[0.0, 0.0], [0.0, 0.0], [-1.0, 1.0]]], dtype=self.dtype)
        self.cylinder = Cylinder(
            name="cylinder",
            end_pts=end_pts,
            linear_vel=torch.zeros(1, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(1, 3, 1, dtype=self.dtype),
            radius=torch.tensor([[[0.5]]], dtype=self.dtype),
            mass=torch.ones(1, 1, 1, dtype=self.dtype),
            sites={}
        )

        rot_mat = torch.eye(3, dtype=self.dtype).unsqueeze(0)
        self.prism = StaticPrism(
            name="prism",
            center=center.clone(),
            rot_mat=rot_mat,
            half_lens=(torch.tensor([[1.0]], dtype=self.dtype),
                       torch.tensor([[1.0]], dtype=self.dtype),
                       torch.tensor([[1.0]], dtype=self.dtype)),
            dtype=self.dtype
        )

        self.rect_plane = StaticRectPlane(
            name="plane",
            center=center.clone(),
            rot_mat=rot_mat.clone(),
            half_lens=(torch.tensor([[2.0]], dtype=self.dtype),
                       torch.tensor([[2.0]], dtype=self.dtype)),
            dtype=self.dtype
        )

    def test_sphere_sphere_dispatch(self):
        """Test dispatch returns sphere_sphere function."""
        fn = get_dist_fn(self.sphere, self.sphere)
        self.assertEqual(fn, sphere_sphere_signed_distance)

    def test_sphere_cylinder_dispatch(self):
        """Test dispatch returns sphere_cylinder function."""
        fn = get_dist_fn(self.sphere, self.cylinder)
        self.assertEqual(fn, sphere_cylinder_signed_distance)

    def test_cylinder_cylinder_dispatch(self):
        """Test dispatch returns cylinder_cylinder function."""
        fn = get_dist_fn(self.cylinder, self.cylinder)
        self.assertEqual(fn, cylinder_cylinder_signed_distance)

    def test_sphere_prism_dispatch(self):
        """Test dispatch returns sphere_static_prism function."""
        fn = get_dist_fn(self.sphere, self.prism)
        self.assertEqual(fn, sphere_static_prism_signed_distance)

    def test_cylinder_prism_dispatch(self):
        """Test dispatch returns cylinder_static_prism function."""
        fn = get_dist_fn(self.cylinder, self.prism)
        self.assertEqual(fn, cylinder_static_prism_signed_distance)

    def test_sphere_rect_plane_dispatch(self):
        """Test dispatch returns sphere_static_rect_plane function."""
        fn = get_dist_fn(self.sphere, self.rect_plane)
        self.assertEqual(fn, sphere_static_rect_plane_signed_distance)

    def test_unsupported_combination_raises(self):
        """Test that unsupported combinations raise ValueError."""
        with self.assertRaises(ValueError):
            get_dist_fn(self.cylinder, self.rect_plane)

    def test_dispatched_fn_is_callable(self):
        """Test that dispatched function can actually be called."""
        # Create a second sphere at a different position
        center2 = torch.tensor([[[5.0], [0.0], [0.0]]], dtype=self.dtype)
        sphere2 = SphereState(
            name="sphere2",
            center=center2,
            linear_vel=torch.zeros(1, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(1, 3, 1, dtype=self.dtype),
            radius=torch.tensor([[1.0]], dtype=self.dtype),
            mass=torch.ones(1, 1, 1, dtype=self.dtype),
            principal_axis=torch.tensor([[[0], [0], [1]]], dtype=self.dtype),
            sites={}
        )

        fn = get_dist_fn(self.sphere, sphere2)
        p1, p2, dist = fn(self.sphere, sphere2)

        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        self.assertAlmostEqual(dist.item(), 3.0, places=4)


class TestEdgeCases(unittest.TestCase):
    """Test edge cases for all functions."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.device = torch.device('cpu')
        self.dtype = DEFAULT_DTYPE
    
    def create_sphere(self, center, radius, name="sphere"):
        """Helper to create a sphere."""
        batch_size = center.shape[0] if len(center.shape) > 1 else 1
        if len(center.shape) == 1:
            center = center.unsqueeze(0).unsqueeze(-1)
        elif len(center.shape) == 2:
            center = center.unsqueeze(-1)
        
        if isinstance(radius, (int, float)):
            radius = torch.tensor([[radius]], dtype=self.dtype)
        elif len(radius.shape) == 0:
            radius = radius.unsqueeze(0).unsqueeze(0)
        elif len(radius.shape) == 1:
            radius = radius.unsqueeze(-1)
        
        return SphereState(
            name=name,
            center=center,
            linear_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            ang_vel=torch.zeros(batch_size, 3, 1, dtype=self.dtype),
            radius=radius,
            mass=torch.ones(batch_size, 1, 1, dtype=self.dtype),
            principal_axis=torch.tensor([[0, 0, 1]], dtype=self.dtype).repeat(batch_size, 1, 1),
            sites={}
        )
    
    def test_coincident_spheres(self):
        """Test two spheres at the same location."""
        sphere1 = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        sphere2 = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 1.0)
        
        p1, p2, dist = sphere_sphere_signed_distance(sphere1, sphere2)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Distance should be negative (fully penetrating)
        self.assertLess(dist.item(), 0)
        # Expected: 0 - 1 - 1 = -2
        self.assertAlmostEqual(dist.item(), -2.0, places=5)
    
    def test_very_small_spheres(self):
        """Test with very small radii."""
        sphere1 = self.create_sphere(torch.tensor([[0.0, 0.0, 0.0]], dtype=self.dtype), 0.001)
        sphere2 = self.create_sphere(torch.tensor([[0.01, 0.0, 0.0]], dtype=self.dtype), 0.001)
        
        p1, p2, dist = sphere_sphere_signed_distance(sphere1, sphere2)
        
        # Check output shapes
        self.assertEqual(p1.shape, (1, 3, 1))
        self.assertEqual(p2.shape, (1, 3, 1))
        self.assertEqual(dist.shape, (1, 1))
        
        # Should still work without numerical issues
        self.assertIsInstance(dist.item(), float)


if __name__ == '__main__':
    unittest.main()
