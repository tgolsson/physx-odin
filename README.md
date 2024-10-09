<div align="center">

# 🎳 physx-odin

**Unsafe automatically-generated Odin bindings for [NVIDIA PhysX 5.1](https://github.com/NVIDIA-Omniverse/PhysX) C++ API.**

</div>

This is a fork from [`physx-rs`](https://github.com/EmbarkStudios/physx-rs), using the same methodology for binding
generation. Check out the [`physx-sys`](https://github.com/EmbarkStudios/physx-rs/tree/main/physx-sys) and related
presentation for more information.

## Basic usage

```odin
unsafe {
    let foundation = physx_create_foundation();
    let physics = physx_create_physics(foundation);

    let mut scene_desc = PxSceneDesc_new(PxPhysics_getTolerancesScale(physics));
    scene_desc.gravity = PxVec3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    };

    let dispatcher = phys_PxDefaultCpuDispatcherCreate(
        1,
        null_mut(),
        PxDefaultCpuDispatcherWaitForWorkMode::WaitForWork,
        0,
    );
    scene_desc.cpuDispatcher = dispatcher.cast();
    scene_desc.filterShader = get_default_simulation_filter_shader();

    let scene = PxPhysics_createScene_mut(physics, &scene_desc);

    // Your physics simulation goes here
}
```

## Examples

### [Ball](examples/ball.rs)

A simple example to showcase how to use physx-sys. It can be run with `odin run examples/ball.odin -file`.

```txt
Simulated 100 timesteps in 8.161109 ms.
Heights over time (x=time, dt=100ms)

                  o

                   o

                    o                   ooooooo
                                      oo       ooo
                     o               o            o
                                   oo              o
                                  o                 o
                      o          o                   o
                                o                     o
                       o       o                       o
                                                        o           ooooooo
                        o     o                          o        oo       oo
                             o                                  oo           oo
                            o                             o    o               o    ooooooooo
                         o o                               o oo                 oooo         ooooooo
```

## License

Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

Note that the [PhysX C++ SDK](https://github.com/NVIDIA-Omniverse/PhysX) has its [own BSD 3 license](LICENSE-BSD).

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the
Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
