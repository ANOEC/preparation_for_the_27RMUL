import("core.project.config")

function codec(target, root)
    local serial = path.join(root, "utility/serialization")
    target:add("includedirs", path.join(serial, "include"), {public = true})
    if (config.get("wire_backend") or "zpp") == "zpp" then
        target:add("sysincludedirs", path.join(serial, "third_party/zpp"), {public = true})
        target:add("defines", "WIRE_BACKEND_ZPP=1", {public = true})
    else
        target:add("sysincludedirs", path.join(serial, "third_party/pfr/include"), {public = true})
        target:add("defines", "WIRE_BACKEND_PFR=1", "BOOST_PFR_USE_CPP17=1",
                   "BOOST_PFR_USE_CPP26=0", "BOOST_PFR_USE_LOOPHOLE=0", {public = true})
    end
end
