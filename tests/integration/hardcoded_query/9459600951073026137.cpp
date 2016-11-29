#include <iostream>
#include <string>

#include "query/util.hpp"
#include "query/i_plan_cpu.hpp"
#include "storage/model/properties/all.hpp"
#include "using.hpp"

using std::cout;
using std::endl;

// Query: MATCH (p:profile {profile_id: 111, partner_id:55})-[s:score]-(g.garment {garment_id: 1234}) DELETE s
// Hash: 9459600951073026137

class CodeCPU : public IPlanCPU<Stream>
{
public:

    bool run(Db &db, plan_args_t &args, Stream &stream) override
    {
        DbAccessor t(db);

        auto &profile = t.label_find_or_create("profile");
        auto &score   = t.type_find_or_create("score");
        auto &garment = t.label_find_or_create("garment");

        indices_t profile_ind = {{"profile_id", 0}, {"partner_id", 1}};
        indices_t garment_ind = {{"garment_id", 2}};

        auto profile_prop = query_properties(profile_ind, args);
        auto garment_prop = query_properties(garment_ind, args);

        auto score_key = t.edge_property_key("score", args[3].key.flags());

        // TODO: decide path (which index is better)
        //       3 options p->s->g, g->s->p, g<-s->p
        //       NOTE! both direections have to be chacked
        //       because pattern is non directional
        //       OR
        //       even better, use index on label and property

        // just one option p->s->g!
        Option<const EdgeAccessor> e1;
        profile.index()
            .for_range(t)
            .properties_filter(t, profile_prop)
            .out()
            .type(score)
            .clone_to(e1)
            .to()
            .label(garment)
            .properties_filter(t, garment_prop)
            .for_all([&](auto va) -> void {
                auto ea = e1.get().update();
                ea.remove();
            });

        Option<const EdgeAccessor> e2;
        profile.index()
            .for_range(t)
            .properties_filter(t, profile_prop)
            .in()
            .type(score)
            .clone_to(e1)
            .from()
            .label(garment)
            .properties_filter(t, garment_prop)
            .for_all([&](auto va) -> void {
                auto ea = e2.get().update();
                ea.remove();
            });

        stream.write_empty_fields();
        stream.write_meta("w");

        return t.commit();

    }

    ~CodeCPU() {}
};

extern "C" IPlanCPU<Stream>* produce()
{
    return new CodeCPU();
}

extern "C" void destruct(IPlanCPU<Stream>* p)
{
    delete p;
}
